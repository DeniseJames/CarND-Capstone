import numpy as np
import tensorflow as tf
from collections import Counter

from styx_msgs.msg import TrafficLight


def load_image_into_numpy_array(cv_image):
    img = np.asarray(cv_image[:, :]).astype(np.uint8)
    return np.expand_dims(img, axis=0)


class DeepDetector(object):
    def __init__(self, model_path):
        # Load TensorFlow model into memory
        detection_graph = tf.Graph()
        with detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(model_path, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

        self.sess = tf.Session(graph=detection_graph)
        self.image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
        self.detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
        self.detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')

    def get_light_state(self, image, min_thresh=0.5):
        """Detect and determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image from front camera
            min_thresh (double): threshold for a detected target to be effective

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        image = load_image_into_numpy_array(image)
        (scores, classes) = self.sess.run([self.detection_scores, self.detection_classes],
                                          feed_dict={self.image_tensor: image})
        classes = np.squeeze(classes).astype(np.int32)
        scores = np.squeeze(scores)
        hist = []
        for i, s in enumerate(scores):
            if s > min_thresh:
                hist.append(classes[i])
        if hist:
            ctr = Counter(hist)
            return ctr.most_common(1)[0][0]-1
        return TrafficLight.UNKNOWN

    def close(self):
        self.sess.close()
