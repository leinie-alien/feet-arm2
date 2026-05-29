import sys
path = "/home/zyy/arm/task3_ws_2/src/my_pick_pipeline/my_pick_pipeline/realtime_inference.py"
with open(path, 'r') as f:
    text = f.read()

old_block = """        if global_state['success'] and global_state['pose'] is not None:
            response.pick_pose = global_state['pose']
            node.get_logger().info(f'Service called for {request.object_name}: returned valid pose.')
        else:
            response.pick_pose = PoseStamped() # 返回空位姿
            node.get_logger().warn(f'Service called for {request.object_name}: target not found.')"""

new_block = """        if global_state['success'] and global_state['pose'] is not None:
            response.pick_pose = global_state['pose']
            response.success = True
            node.get_logger().info(f'Service called for {request.object_name}: returned valid pose.')
        else:
            response.pick_pose = PoseStamped() # 返回空位姿
            response.success = False
            node.get_logger().warn(f'Service called for {request.object_name}: target not found.')"""

text = text.replace(old_block, new_block)
with open(path, 'w') as f:
    f.write(text)
print("done")
