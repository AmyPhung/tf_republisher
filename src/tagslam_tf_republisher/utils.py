import yaml
from geometry_msgs.msg import Transform

def save_tf(tf, filename):
	data = {
        "translation": {
            "x": tf.translation.x,
            "y": tf.translation.y,
            "z": tf.translation.z
        },
        "rotation": {
            "x": tf.rotation.x,
            "y": tf.rotation.y,
            "z": tf.rotation.z,
            "w": tf.rotation.w
        }
	}

	with open(filename, 'w') as outfile:
		yaml.dump(data, outfile)


def load_tf(filename):
    data = {}
    with open(filename, 'r') as stream:
        data=yaml.safe_load(stream)
        
    tf = Transform()
    tf.translation.x = data["translation"]["x"]
    tf.translation.y = data["translation"]["y"]
    tf.translation.z = data["translation"]["z"]

    tf.rotation.x = data["rotation"]["x"]
    tf.rotation.y = data["rotation"]["y"]
    tf.rotation.z = data["rotation"]["z"]
    tf.rotation.w = data["rotation"]["w"]

    return tf
