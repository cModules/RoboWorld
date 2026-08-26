import sys
import os

# Add examples/gui_demo to path
sys.path.insert(0, 'examples/gui_demo')

# Redirect stdout/stderr to file
sys.stdout = open('test_output.log', 'w')
sys.stderr = open('test_errors.log', 'w')

try:
    from python_viewer import RobotRenderer
    
    # Create renderer and load URDF
    renderer = RobotRenderer()
    result = renderer.load_urdf('models/lerobot/so101/urdf/so101.urdf')
    
    print(f"Load result: {result}")
    print(f"Links: {len(renderer.links)}")
    print(f"Joints: {len(renderer.joints)}")
    print(f"Root link: {renderer.root_link}")
    print(f"Meshes: {len(renderer.meshes)}")
    
    # Check if meshes are loaded
    for name, triangles in renderer.meshes.items():
        print(f"  {name}: {len(triangles)} triangles")
        
except Exception as e:
    import traceback
    print(f"Error: {e}")
    traceback.print_exc()

sys.stdout.close()
sys.stderr.close()
