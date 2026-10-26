"""Hardware-free checks of installed Xacro and Python launch glue; never executes nodes."""
from pathlib import Path
from tempfile import TemporaryDirectory
import contextlib,io,math,xml.etree.ElementTree as ET,importlib.util
from unittest.mock import patch
import yaml,xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
fixtures=TemporaryDirectory(prefix='lidar-model-check-')
root=Path(fixtures.name); desc=Path(get_package_share_directory('description')); bringup=Path(get_package_share_directory('bringup'))
model=desc/'urdf/robot.urdf.xacro'; config=yaml.safe_load((bringup/'config/lidar_mounts.yaml').read_text())
def expand(values=None,enabled=True):
    f=root/'fixture.yaml'; f.write_text(yaml.safe_dump(config if values is None else values))
    return ET.fromstring(xacro.process_file(str(model),mappings={'with_lidars':str(enabled).lower(),'lidar_mounts':str(f),'hardware_map':str(bringup/'config/hardware_map.yaml')}).toxml())
base=expand(enabled=False)
assert not any(e.attrib.get('name','').startswith('lidar_') for e in base)
assert not any(e.attrib.get('name') in ('base_scan','scan_joint') for e in base)
assert len(base.findall('ros2_control'))==2
valid=config|dict(plate_x=.03,plate_y=-.02,plate_yaw=.4,front_yaw=math.pi,rear_yaw=0.)
dual=expand(valid)
for tag in ('link','joint'):
    names=[e.attrib['name'] for e in dual.findall(tag)]; assert len(names)==len(set(names))
assert len(dual.findall('ros2_control'))==2
for name,parent,child,xyz,yaw in [
 ('lidar_plate_joint','base_link','lidar_plate_link',(.03,-.02,-.005),.4),
 ('lidar_front_joint','lidar_plate_link','lidar_front_link',(.110,.005,.0265),math.pi),
 ('lidar_rear_joint','lidar_plate_link','lidar_rear_link',(-.110,.005,.0265),0.)]:
    joint=dual.find(f"joint[@name='{name}']"); assert joint.attrib['type']=='fixed'
    assert joint.find('parent').attrib['link']==parent and joint.find('child').attrib['link']==child
    actual=list(map(float,joint.find('origin').attrib['xyz'].split()))
    assert all(math.isclose(a,b,abs_tol=1e-10) for a,b in zip(actual,xyz))
    assert math.isclose(float(joint.find('origin').attrib['rpy'].split()[2]),yaw)
# Apart from explicit LiDAR links/joints, model remains identical across the flag.
assert [ET.tostring(e) for e in base]==[ET.tostring(e) for e in dual if not e.attrib.get('name','').startswith('lidar_')]
cases=[config,valid|{'h_scan':-.001}]
for key in config:
    for bad in (None,float('nan'),float('inf'),'0',True): cases.append(valid|{key:bad})
for values in cases:
    try:
        with contextlib.redirect_stderr(io.StringIO()): expand(values)
    except Exception: pass
    else: raise AssertionError(values)
spec=importlib.util.spec_from_file_location('lidar_launch',bringup/'launch/lidar.launch.py'); launch=importlib.util.module_from_spec(spec); spec.loader.exec_module(launch)
def ctx(**kw):
    c=LaunchContext(); c.launch_configurations.update(dict(driver_variant='experimental_stationary',stationary_test='true',driver_config=str(bringup/'config/lidar_drivers.yaml'),publish_model='false',lidar_mounts=str(bringup/'config/lidar_mounts.yaml'),front_port='/dev/test_front',rear_port='/dev/test_rear',rviz_fixed_frame='base_link',enable_front='true',enable_rear='true',use_rviz='false')|kw); return c
assert len(launch.setup(ctx()))==3
for kw in ({'stationary_test':'false'},{'publish_model':'true'}):
    with patch.object(launch,'Node') as node:
        try:
            with contextlib.redirect_stderr(io.StringIO()): launch.setup(ctx(**kw))
        except Exception: pass
        else: raise AssertionError(kw)
        node.assert_not_called()
expand(valid)
with patch.object(launch,'Node',side_effect=lambda **kw:kw):
    nodes=launch.setup(ctx(publish_model='true',lidar_mounts=str(root/'fixture.yaml')))
    assert [n['package'] for n in nodes]==['robot_state_publisher','coin_d4_experimental_driver','coin_d4_experimental_driver','rviz2']
    for side,node in zip(('front','rear'),nodes[1:3]):
        assert node['namespace']=='lidar_'+side
        assert node['parameters'][1]=={'port':'/dev/test_'+side,'stationary_test':True}
print(f'PASS: default and synthetic dual URDF, unique frames, TF transforms, unchanged non-LiDAR model, 2 control systems, {len(cases)} invalid geometry cases, raw/model launch declarations and fail-before-node guards. No actions executed.')

fixtures.cleanup()
