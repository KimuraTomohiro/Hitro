import rospy
import tf
from geometry_msgs.msg import Point, Quaternion, TransformStamped

rospy.init_node('tf_example')

# 例として、新しいトランスフォーメーションを計算するためのデータを作成します
translation = Point(x=1.0, y=2.0, z=3.0)
rotation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
child_frame_id = 'base_link'
parent_frame_id = 'hand_roll_1'

# 新しいトランスフォーメーションを作成します
new_transform = TransformStamped()
new_transform.header.stamp = rospy.Time.now()
new_transform.header.frame_id = parent_frame_id
new_transform.child_frame_id = child_frame_id
new_transform.transform.translation = translation
new_transform.transform.rotation = rotation

# トランスフォーメーションを計算します
listener = tf.TransformerROS()
listener.setTransform(new_transform)

# 特定のフレーム間のトランスフォーメーションを取得します
target_frame_id = 'hand_roll_1'
source_frame_id = 'base_link'
try:
    (trans, rot) = listener.lookupTransform(target_frame_id, source_frame_id, rospy.Time(0))
    print("Translation:", trans)
    print("Rotation:", rot)
except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    print("Failed to lookup transform.")
