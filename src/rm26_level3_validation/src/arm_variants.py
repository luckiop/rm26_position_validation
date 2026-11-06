#!/usr/bin/env python3
"""Freeze an RM26 arm and optionally remove a middle section of a supported link."""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import shutil
import struct
import xml.etree.ElementTree as ET


def file_hash(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def dump(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')


# Projection masks retain offsets perpendicular to each rod's axial direction.
LINK_PROFILES = {
    'j1_link': {'child': 'j2_link', 'axis_mask': (1, 1, 0), 'end_clearance_m': .06},
    'j4_link': {'child': 'j5_link', 'axis_mask': (1, 0, 0), 'end_clearance_m': .018},
    'j6_link': {'terminal_reference_m': (0, 0, .1737941210), 'axis_mask': (0, 0, 1),
                'end_clearance_m': .025},
    'j5_link': {'child': 'j6_link', 'axis_mask': (0, 0, 1), 'end_clearance_m': .012},
}


def validate_variant(spec):
    changes = spec.get('changes', [])
    if not isinstance(changes, list) or len(changes) > len(LINK_PROFILES):
        raise ValueError('Generator supports up to four link changes')
    seen_links = set()
    for change in changes:
        if not isinstance(change, dict):
            raise ValueError('Each link change must be an object')
        if change.get('link') not in LINK_PROFILES or change.get('method') not in ('remove_middle', 'stretch_axial'):
            raise ValueError('Only j1_link, j4_link, j5_link or j6_link / remove_middle or stretch_axial is supported')
        if change['link'] in seen_links:
            raise ValueError(f"A variant may change each link only once: {change['link']}")
        seen_links.add(change['link'])
        if change['method'] == 'remove_middle':
            distance_keys = ('shorten_m', 'shorten_fraction')
        else:
            distance_keys = ('lengthen_m', 'lengthen_fraction')
        if sum(key in change for key in distance_keys) != 1:
            raise ValueError('Specify exactly one absolute or fractional length change')
        if change['method'] == 'stretch_axial' and 'cut_center_m' in change:
            raise ValueError('cut_center_m applies only to remove_middle')
        for key in (*distance_keys, 'cut_center_m'):
            if key not in change:
                continue
            value = change[key]
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f'{key} must be finite and positive')
        fraction_key = 'shorten_fraction' if change['method'] == 'remove_middle' else 'lengthen_fraction'
        if fraction_key in change and change[fraction_key] >= 1:
            raise ValueError(f'{fraction_key} must be in (0, 1)')
    return changes


def variant_label(spec):
    changes = validate_variant(spec)
    if not changes:
        return '原始'
    labels = []
    for change in changes:
        link = change['link'].split('_')[0].upper()
        fraction_key = 'shorten_fraction' if change['method'] == 'remove_middle' else 'lengthen_fraction'
        amount_key = 'shorten_m' if change['method'] == 'remove_middle' else 'lengthen_m'
        amount = (f"{change[fraction_key] * 100:g}%" if fraction_key in change
                  else f"{change[amount_key] * 100:g}cm")
        action = '缩短' if change['method'] == 'remove_middle' else '加长'
        labels.append(link + action + amount)
    return ' + '.join(labels)


def profile_axis(root, link, profile):
    """Return a supported rod's local axial direction and original length."""
    import numpy as np
    body = root.find(f".//body[@name='{link}']")
    if body is None:
        raise ValueError(f'Could not find supported body {link}')
    child = body.find(f"body[@name='{profile['child']}']") if 'child' in profile else None
    reference = (np.fromstring(child.get('pos'), sep=' ') if child is not None
                 else np.asarray(profile['terminal_reference_m'], dtype=float))
    axis = reference * np.asarray(profile['axis_mask'])
    axial_length = float(np.linalg.norm(axis))
    if not math.isfinite(axial_length) or axial_length <= 0:
        raise ValueError(f'Invalid axial length for {link}')
    return body, child, reference, axis / axial_length, axial_length


def requested_length(change, axial_length):
    """Convert a variant's absolute or fractional change to metres."""
    if change['method'] == 'remove_middle':
        return (float(change['shorten_m']) if 'shorten_m' in change
                else float(axial_length * change['shorten_fraction']))
    return (float(change['lengthen_m']) if 'lengthen_m' in change
            else float(axial_length * change['lengthen_fraction']))


def load_triangles(path):
    """Read source binary STL or simple OBJ, preserving triangle winding."""
    import numpy as np
    if path.suffix.lower() == '.stl':
        data = path.read_bytes()
        count = struct.unpack_from('<I', data, 80)[0]
        if len(data) != 84 + count * 50:
            raise ValueError(f'Expected binary STL: {path}')
        return np.array([struct.unpack_from('<9f', data, 96 + 50 * i) for i in range(count)]).reshape(-1, 3, 3)
    vertices, triangles = [], []
    for line in path.read_text().splitlines():
        fields = line.split()
        if not fields:
            continue
        if fields[0] == 'v':
            vertices.append([float(x) for x in fields[1:4]])
        elif fields[0] == 'f':
            indices = [int(f.split('/')[0]) for f in fields[1:]]
            indices = [i - 1 if i > 0 else len(vertices) + i for i in indices]
            triangles.extend([[vertices[indices[0]], vertices[indices[i]], vertices[indices[i+1]]]
                              for i in range(1, len(indices)-1)])
    if not triangles:
        raise ValueError(f'No triangles: {path}')
    return np.asarray(triangles, dtype=float)


def clip_triangles(triangles, axis, offset, positive):
    """Clip each triangle to one half space; split surviving polygons into triangles."""
    import numpy as np
    output = []
    for triangle in triangles:
        polygon = []
        for a, b in zip(triangle, np.roll(triangle, -1, axis=0)):
            da, db = float(a @ axis - offset), float(b @ axis - offset)
            inside_a = da >= -1e-12 if positive else da <= 1e-12
            inside_b = db >= -1e-12 if positive else db <= 1e-12
            if inside_a:
                polygon.append(a)
            if inside_a != inside_b:
                polygon.append(a + da / (da - db) * (b - a))
        for i in range(1, len(polygon) - 1):
            face = np.array([polygon[0], polygon[i], polygon[i+1]])
            if np.linalg.norm(np.cross(face[1]-face[0], face[2]-face[0])) > 1e-15:
                output.append(face)
    return np.asarray(output, dtype=float).reshape(-1, 3, 3)


def convex_surface(triangles):
    """Close a clipped original convex collision piece without joining separate pieces."""
    import numpy as np
    from scipy.spatial import ConvexHull, QhullError
    vertices = np.unique(np.round(triangles.reshape(-1, 3), 12), axis=0)
    if len(vertices) < 4:
        return np.empty((0, 3, 3))
    try:
        hull = ConvexHull(vertices)
    except QhullError as error:
        raise ValueError('Degenerate clipped collision piece; adjust cut planes') from error
    faces = vertices[hull.simplices].copy()
    for face, equation in zip(faces, hull.equations):
        if np.dot(np.cross(face[1]-face[0], face[2]-face[0]), equation[:3]) < 0:
            face[[1, 2]] = face[[2, 1]]
    return faces


def write_obj(path, triangles):
    import numpy as np
    vertices, indices = np.unique(triangles.reshape(-1, 3), axis=0, return_inverse=True)
    with path.open('w') as stream:
        stream.write('# Generated rod variant geometry; lengths in m\n')
        for vertex in vertices:
            stream.write('v ' + ' '.join(f'{x:.12g}' for x in vertex) + '\n')
        for face in indices.reshape(-1, 3) + 1:
            stream.write('f ' + ' '.join(map(str, face)) + '\n')


def generate_arm(source, spec, output):
    """Create a self-contained immutable-by-contract model package and hash manifest."""
    changes = validate_variant(spec)
    source = Path(source).resolve(strict=True)
    output = Path(output).resolve()
    if output.exists():
        raise ValueError(f'Arm output must not exist: {output}')
    tree = ET.parse(source)
    root = tree.getroot()
    compiler = root.find('compiler')
    meshdir = (source.parent / compiler.get('meshdir', '')).resolve()
    assets = root.find('asset')
    # Reject unhandled external resources instead of silently hashing an incomplete bundle.
    if root.findall('include') or any(a.tag != 'mesh' for a in assets):
        raise ValueError('Expected standalone arm XML with mesh assets only')
    if changes:
        import numpy as np
        profile_lengths = {link: profile_axis(root, link, profile)[4]
                           for link, profile in LINK_PROFILES.items()}
        total_original_length = sum(profile_lengths.values())
        total_delta = sum(requested_length(change, profile_lengths[change['link']]) *
                          (1 if change['method'] == 'stretch_axial' else -1)
                          for change in changes)
        if abs(total_delta) > 0.30 * total_original_length + 1e-12:
            raise ValueError('Net total rod-length change must stay within ±30%')
    output.mkdir(parents=True)
    (output / 'meshes').mkdir()
    source_files = {str(source): file_hash(source)}
    paths = {}
    for index, mesh in enumerate(assets.findall('mesh')):
        path = (meshdir / mesh.get('file')).resolve(strict=True)
        source_files[str(path)] = file_hash(path)
        destination = f'meshes/{index:04d}{path.suffix.lower()}'
        shutil.copyfile(path, output / destination)
        mesh.set('file', destination)
        paths[mesh.get('name')] = path
    compiler.set('meshdir', '.')
    modifications = []
    attach_link = 'j6_link'

    for change in changes:
        profile = LINK_PROFILES[change['link']]
        body, child, before, axis, axial_length = profile_axis(root, change['link'], profile)
        length = requested_length(change, axial_length)
        shortening = change['method'] == 'remove_middle'
        delta = -length if shortening else length
        after = before + delta * axis
        if child is not None:
            child.set('pos', ' '.join(f'{x:.12g}' for x in after))
            mount_record = {'child': profile['child'], 'child_position_before_m': before.tolist(),
                            'child_position_after_m': after.tolist()}
        else:
            # Keep the held unit aligned with a shortened or lengthened terminal link.
            attach_link = 'variant_energy_mount'
            shift = delta * axis
            ET.SubElement(body, 'body', name=attach_link, pos=' '.join(f'{x:.12g}' for x in shift))
            mount_record = {'terminal_reference_before_m': before.tolist(),
                            'terminal_reference_after_m': after.tolist(), 'energy_mount_body': attach_link,
                            'energy_mount_shift_m': shift.tolist()}

        counts = {'visual_triangles': 0, 'collision_pieces': 0}
        for geom in list(body.findall('geom')):
            mesh_name = geom.get('mesh')
            if mesh_name not in paths or geom.get('pos', '0 0 0') != '0 0 0' or geom.get('quat', '1 0 0 0') != '1 0 0 0':
                raise ValueError('Generator expects untransformed link mesh geometry')
            triangles = load_triangles(paths[mesh_name])
            collision = geom.get('class') == 'collision'
            original = assets.find(f"mesh[@name='{mesh_name}']")
            if original.get('scale', '1 1 1') != '1 1 1':
                raise ValueError('Scaled source mesh unsupported')
            body.remove(geom)
            if shortening:
                center = change.get('cut_center_m', float(axial_length / 2))
                lower, upper = center - length / 2, center + length / 2
                clearance = profile['end_clearance_m']
                if lower <= clearance or upper >= axial_length - clearance:
                    raise ValueError(f'Cut intersects protected {clearance:g} m joint-end zones')
                pieces = [('prox', clip_triangles(triangles, axis, lower, False)),
                          ('dist', clip_triangles(triangles, axis, upper, True))]
            else:
                # Stretch axial coordinates from the proximal origin; this preserves
                # each source collision piece instead of opening an unmodeled gap.
                projection = triangles @ axis
                stretched = triangles + (np.maximum(projection, 0.0) * (length / axial_length))[..., None] * axis
                pieces = [('stretched', stretched)]
            for suffix, clipped in pieces:
                if not len(clipped):
                    continue
                if collision:
                    clipped = convex_surface(clipped)
                    if not len(clipped):
                        continue
                if shortening and suffix == 'dist':
                    clipped -= length * axis
                name = mesh_name + '_' + suffix
                relative = 'meshes/' + name + '.obj'
                write_obj(output / relative, clipped)
                asset = copy.deepcopy(original)
                asset.set('name', name)
                asset.set('file', relative)
                assets.append(asset)
                part = copy.deepcopy(geom)
                part.set('name', geom.get('name') + '_' + suffix)
                part.set('mesh', name)
                body.insert(0, part)
                counts['collision_pieces' if collision else 'visual_triangles'] += 1 if collision else len(clipped)
        record = {**change, 'length_change_m': float(delta), 'axis_in_link': axis.tolist(),
                  **mount_record, 'axial_length_before_m': float(axial_length),
                  'axial_length_after_m': float(axial_length + delta), **counts}
        if shortening:
            record.update(shorten_m=float(length), cut_center_m=float(center),
                          end_clearance_m=clearance, cut_interval_m=[lower, upper])
        else:
            record['lengthen_m'] = float(length)
        modifications.append(record)
    # Strip now-unused source mesh assets and files, keeping only what the model references.
    referenced = {g.get('mesh') for g in root.iter('geom') if g.get('mesh')}
    for mesh in list(assets):
        if mesh.get('name') not in referenced:
            (output / mesh.get('file')).unlink()
            assets.remove(mesh)
    ET.indent(tree, space='  ')
    tree.write(output / 'model.xml', encoding='utf-8', xml_declaration=True)
    dump(output / 'variant_config.json', spec)
    files = {str(p.relative_to(output)): file_hash(p) for p in sorted(output.rglob('*')) if p.is_file()}
    fingerprint = hashlib.sha256(json.dumps(files, sort_keys=True).encode()).hexdigest()
    default_id = ('original' if not changes else '-'.join(
        change['link'].replace('_link', '') +
        ('-shortened' if change['method'] == 'remove_middle' else '-lengthened')
        for change in changes))
    manifest = {'schema_version': 1, 'id': spec.get('id', default_id),
                'label': variant_label(spec), 'attach_link': attach_link, 'fingerprint_sha256': fingerprint,
                'model_file': 'model.xml', 'files_sha256': files, 'source_files_sha256': source_files,
                'modifications': modifications, 'inertia_policy': 'original mass and inertia retained; kinematics/collision exploration only',
                'visual_seam_policy': 'axial cut/stretch geometry; no manufacturing-grade joint or solid validation'}
    dump(output / 'manifest.json', manifest)
    return {'id': manifest['id'], 'label': manifest['label'], 'attach_link': attach_link, 'fingerprint_sha256': fingerprint,
            'model_path': str(output / 'model.xml'), 'manifest_path': str(output / 'manifest.json'),
            'manifest_sha256': file_hash(output / 'manifest.json')}


def verify_arm(identity):
    manifest_path = Path(identity['manifest_path'])
    if identity.get('manifest_sha256') and file_hash(manifest_path) != identity['manifest_sha256']:
        raise ValueError('Arm manifest file fingerprint mismatch')
    manifest = json.loads(manifest_path.read_text())
    files = manifest['files_sha256']
    fingerprint = hashlib.sha256(json.dumps(files, sort_keys=True).encode()).hexdigest()
    if fingerprint != identity['fingerprint_sha256'] or manifest['fingerprint_sha256'] != fingerprint:
        raise ValueError('Arm manifest fingerprint mismatch')
    if identity['label'] != manifest['label'] or identity['id'] != manifest['id']:
        raise ValueError('Arm identity/manifest label mismatch')
    if identity.get('attach_link', 'j6_link') != manifest.get('attach_link', 'j6_link'):
        raise ValueError('Arm attachment identity mismatch')
    expected = (manifest_path.parent / manifest['model_file']).resolve()
    if Path(identity['model_path']).resolve() != expected:
        raise ValueError('Arm model path does not match manifest')
    for relative, digest in files.items():
        path = (manifest_path.parent / relative).resolve()
        if not path.is_relative_to(manifest_path.parent.resolve()) or file_hash(path) != digest:
            raise ValueError(f'Arm resource fingerprint mismatch: {relative}')
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-model', type=Path, required=True)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(generate_arm(args.source_model, json.loads(args.config.read_text()), args.output), ensure_ascii=False))


if __name__ == '__main__':
    main()
