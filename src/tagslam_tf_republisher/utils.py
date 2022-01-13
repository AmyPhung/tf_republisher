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
    data = yaml.load(filename)
    tf = Transform()
    data["translation"]["x"] = tf.translation.x
    data["translation"]["x"] = tf.translation.y
    data["translation"]["x"] = tf.translation.z

    data["rotation"]["x"] = tf.rotation.x
    data["rotation"]["y"] = tf.rotation.y
    data["rotation"]["z"] = tf.rotation.z
    data["rotation"]["w"] = tf.rotation.w

    return tf
