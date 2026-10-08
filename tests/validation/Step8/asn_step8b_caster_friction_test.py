"""Isaac Script Editor: temporary caster-friction A/B experiment.

Pause (do not Stop/reset) before applying or restoring. Run with ACTION='apply',
then Play and use the existing recorder. ACTION='restore' removes only this
anonymous session sublayer. No stage files, wheel settings, or ROS parameters
are saved/modified. Zero friction isolates caster drag; it is not a calibrated
real caster model or a permanent fix.
"""
ACTION = 'apply'  # Change to 'restore' to undo this experiment.

import json
from datetime import datetime, timezone
from pathlib import Path

import omni.timeline
import omni.usd
from pxr import PhysxSchema, Sdf, Usd, UsdPhysics, UsdShade

stage = omni.usd.get_context().get_stage()
if stage is None:
    raise RuntimeError('Open the current scene first.')
if omni.timeline.get_timeline_interface().is_playing():
    raise RuntimeError('Pause Isaac before applying/restoring; do not Stop/reset.')
if ACTION not in ('apply', 'restore'):
    raise ValueError("ACTION must be 'apply' or 'restore'.")

caster_path = '/World/V1_Robot/base_link/collisions/mesh_2'
material_path = '/World/Step8B_CasterExperimentMaterial'
caster = stage.GetPrimAtPath(caster_path)
if not caster.IsValid() or caster.GetTypeName() != 'Sphere':
    raise RuntimeError('Expected the baseline front caster sphere at ' + caster_path)
session = stage.GetSessionLayer()
record = globals().get('_asn_step8b_caster_experiment')


def binding_snapshot():
    material, relationship = UsdShade.MaterialBindingAPI(caster).ComputeBoundMaterial(
        materialPurpose='physics')
    return dict(material=str(material.GetPath()) if material else None,
                relationship=str(relationship.GetPath()) if relationship else None)


metadata = dict(action=ACTION, utc=datetime.now(timezone.utc).isoformat(),
                caster= caster_path, root_layer=stage.GetRootLayer().identifier,
                before_binding=binding_snapshot())

if ACTION == 'apply':
    if record and record['session'] == session.identifier:
        raise RuntimeError('Experiment already active; restore it before applying again.')
    if stage.GetPrimAtPath(material_path).IsValid():
        raise RuntimeError('Experiment material path already exists; refusing to overwrite it.')
    layer = Sdf.Layer.CreateAnonymous('step8b_caster_friction.usda')
    session.subLayerPaths = [layer.identifier] + list(session.subLayerPaths)
    try:
        with Usd.EditContext(stage, layer):
            material = UsdShade.Material.Define(stage, material_path)
            api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
            api.CreateStaticFrictionAttr().Set(0.0)
            api.CreateDynamicFrictionAttr().Set(0.0)
            api.CreateRestitutionAttr().Set(0.0)
            PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim()).CreateFrictionCombineModeAttr().Set('min')
            UsdShade.MaterialBindingAPI.Apply(caster).Bind(
                material, UsdShade.Tokens.strongerThanDescendants, 'physics')
        if binding_snapshot()['material'] != material_path:
            raise RuntimeError('Physics-material binding did not resolve to the experiment material.')
    except BaseException:
        session.subLayerPaths = [p for p in session.subLayerPaths if p != layer.identifier]
        raise
    globals()['_asn_step8b_caster_experiment'] = dict(
        layer=layer, session=session.identifier, before_binding=metadata['before_binding'])
    metadata.update(static_friction=0.0, dynamic_friction=0.0, combine_mode='min',
                    session_sublayer=layer.identifier)
else:
    if not record or record['session'] != session.identifier:
        raise RuntimeError('No matching experiment in this Script Editor session.')
    identifier = record['layer'].identifier
    session.subLayerPaths = [p for p in session.subLayerPaths if p != identifier]
    if binding_snapshot() != record['before_binding']:
        raise RuntimeError('Previous material binding was not restored; inspect other session edits.')
    globals().pop('_asn_step8b_caster_experiment', None)

metadata['after_binding'] = binding_snapshot()
output = Path.home() / 'Downloads' / ('Step8B_caster_' + ACTION + '.json')
output.parent.mkdir(parents=True, exist_ok=True)
output.write_text(json.dumps(metadata, indent=2) + '\n')
print(f'Caster experiment {ACTION} complete. Metadata: {output}')
print('Resume Play; wheel geometry, gains, collision shape, and ROS limits are unchanged.')
