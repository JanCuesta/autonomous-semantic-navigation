"""Run once in Isaac Script Editor, paused, after the temporary caster test.

Save the same idealized zero-friction caster material in the referenced robot
USD asset, then remove the experiment's anonymous session sublayer. The asset
gets a timestamped backup. This does not edit the map/world file or ROS.
"""

from datetime import datetime, timezone
from pathlib import Path
import shutil

import omni.timeline
import omni.usd
from pxr import PhysxSchema, Usd, UsdPhysics, UsdShade

if omni.timeline.get_timeline_interface().is_playing():
    raise RuntimeError('Pause the timeline before persisting caster physics.')

world = omni.usd.get_context().get_stage()
if world is None:
    raise RuntimeError('Open Testing_World+Robot.usd in Isaac first.')
root = Path(world.GetRootLayer().realPath)
if root.name != 'Testing_World+Robot.usd':
    raise RuntimeError('Expected the Testing_World+Robot.usd scene.')
asset_path = (root.parent.parent / 'robots' / 'V1_Robot.usd').resolve()
if not asset_path.is_file():
    raise RuntimeError('Robot asset not found: ' + str(asset_path))

world_caster = world.GetPrimAtPath('/World/V1_Robot/base_link/collisions/mesh_2')
if not world_caster.IsValid() or world_caster.GetTypeName() != 'Sphere':
    raise RuntimeError('Unexpected world caster prim.')

def bound_path(prim):
    material, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial(
        materialPurpose='physics')
    return str(material.GetPath()) if material else None

experiment_material = '/World/Step8B_CasterExperimentMaterial'
if bound_path(world_caster) != experiment_material:
    raise RuntimeError('Temporary caster experiment is not bound; stop and inspect.')
session = world.GetSessionLayer()
experiment_layers = [p for p in session.subLayerPaths
                     if p.endswith(':step8b_caster_friction.usda')]
if len(experiment_layers) != 1:
    raise RuntimeError('Expected exactly one temporary caster experiment layer.')

robot = Usd.Stage.Open(str(asset_path))
if not robot:
    raise RuntimeError('Could not open robot asset.')
casters = [prim for prim in robot.Traverse()
           if prim.GetPath().pathString.endswith('/base_link/collisions/mesh_2')]
if len(casters) != 1 or casters[0].GetTypeName() != 'Sphere':
    raise RuntimeError('Could not identify the unique sphere caster in the robot asset.')
caster = casters[0]
material_path = str(caster.GetPath().GetParentPath()) + '/IdealRollingCasterMaterial'
if robot.GetPrimAtPath(material_path).IsValid():
    raise RuntimeError('The permanent caster material already exists; no files modified.')
if bound_path(caster) is not None:
    raise RuntimeError('Caster already has a physics material in the asset.')

stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
backup = asset_path.with_name(asset_path.name + '.before_caster_' + stamp + '.bak')
shutil.copy2(asset_path, backup)

with Usd.EditContext(robot, robot.GetRootLayer()):
    material = UsdShade.Material.Define(robot, material_path)
    api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    api.CreateStaticFrictionAttr().Set(0.0)
    api.CreateDynamicFrictionAttr().Set(0.0)
    api.CreateRestitutionAttr().Set(0.0)
    PhysxSchema.PhysxMaterialAPI.Apply(
        material.GetPrim()).CreateFrictionCombineModeAttr().Set('min')
    UsdShade.MaterialBindingAPI.Apply(caster).Bind(
        material, UsdShade.Tokens.strongerThanDescendants, 'physics')

if bound_path(caster) != material_path:
    raise RuntimeError('Robot-asset material did not resolve; backup: ' + str(backup))
if not robot.GetRootLayer().Save():
    raise RuntimeError('Robot-asset save failed; backup: ' + str(backup))

# The anonymous session layer has priority over the robot asset. Remove only
# the identified test layer so that the newly saved binding can be checked.
session.subLayerPaths = [p for p in session.subLayerPaths
                         if p != experiment_layers[0]]
expected_world_material = '/World/V1_Robot' + material_path[len(str(caster.GetPath()).split('/base_link')[0]):]
actual_world_material = bound_path(world_caster)
if actual_world_material != expected_world_material:
    raise RuntimeError('Asset saved, but world binding needs inspection. '
                       f'Found {actual_world_material}; expected {expected_world_material}. '
                       'Backup: ' + str(backup))

globals().pop('_asn_step8b_caster_experiment', None)
print('Saved robot caster material:', material_path)
print('Resolved world physics material:', actual_world_material)
print('Backup of original robot asset:', backup)
print('Experiment session layer removed. Reopen scene later to verify persistence.')
