#!/usr/bin/env python3
import cv2 as cv
import numpy as np
import tensorflow as tf
from timeit import default_timer as timer

start = timer()
# Replace as needed
MODEL_PATH = "./models/droneInfGraph401092/frozen_inference_graph.pb"
TEST_IMG_PATH = "./DronePics3/"
RESULT_IMG_PATH = "./AIResults/"
NUMBER_OF_IMAGES_PLUS_ONE = 23

def wrap_frozen_graph(graph_def, inputs, outputs):
    def _imports_graph_def():
        tf.compat.v1.import_graph_def(graph_def, name="")

    wrapped_import = tf.compat.v1.wrap_function(_imports_graph_def, [])
    import_graph = wrapped_import.graph
    return wrapped_import.prune(
        tf.nest.map_structure(import_graph.as_graph_element, inputs),
        tf.nest.map_structure(import_graph.as_graph_element, outputs),
    )


def main():
    # print("Loading model...")
    graph_def = tf.compat.v1.GraphDef()
    graph_def.ParseFromString(open(MODEL_PATH, "rb").read())

    # print("Creating frozen func...")
    imagenet_func = wrap_frozen_graph(
        graph_def,
        inputs="image_tensor:0",
        outputs=[
            "detection_classes:0",
            "detection_boxes:0",
            "detection_scores:0",
            "detection_multiclass_scores",
        ],
    )

    # print("*" * 50)
    # print("Frozen model inputs: ")
    # print(imagenet_func.inputs)
    # print("Frozen model outputs: ")
    # print(imagenet_func.outputs)
    
#    keep_looping = True
#    print("starting while loop")
#    while(keep_looping == True):    
#     print("starting for loop")
    for i in range(1, NUMBER_OF_IMAGES_PLUS_ONE):
        # print("*" * 50)
        # print("grab another image %d" %(i))
    
        # Load an image from the dataset
        # print("Running inference...")
        
        image = tf.keras.preprocessing.image.load_img(
            TEST_IMG_PATH + 'test' + str(i) + '.png', grayscale=False, color_mode="rgb", target_size=None
        )
        input_arr = tf.keras.preprocessing.image.img_to_array(image)
        input_arr = np.array([input_arr])
        input_arr = tf.convert_to_tensor(input_arr, dtype=tf.uint8)
        image_shape = input_arr.shape[1:3]
    
        # Run inference
        classes, boxes, scores, multiclass_scores = imagenet_func(input_arr)
        classes, boxes, scores, multiclass_scores = imagenet_func(input_arr)
        boxes = boxes.numpy()
        best_bound = np.multiply(
            boxes[0, 0, 0:4],
            np.array([image_shape[0], image_shape[1], image_shape[0], image_shape[1]]),
        )
        best_bound = best_bound.astype(int)
        # print(
        #     "Bounding Box [Left, Top, Right, Bottom]: ",
        #     best_bound,
        # )
        scores = scores.numpy()
        class_prob = round(scores[0, 0] * 100, 2)
        # print("Score: ", class_prob, "%")
    
        # Write the results to an image file
        output_image = cv.imread(TEST_IMG_PATH + 'test' + str(i) + '.png', cv.IMREAD_COLOR)
        # output_image = cv.imread('test1.png', cv.IMREAD_COLOR)
        cv.rectangle(
            output_image,
            (best_bound[1], best_bound[0]),
            (best_bound[3], best_bound[2]),
            (255, 0, 0),
            3,
        )
        cv.putText(
            output_image,
            #f"{class_prob}%", #because older version of python, f-strings not a thingsud
	    str(class_prob)+"%",
            (best_bound[1], best_bound[0]),
            cv.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
        )
        cv.imwrite(RESULT_IMG_PATH + 'AI' + str(i) + '.png', output_image)
        cv.imshow('output_image', output_image)
        cv.waitKey(3000)
        key = cv.waitKey(1) & 0xFF
        if (key == ord('x')):
            keep_looping = False
    
    #gwpy
    # print("Done")
    cv.destroyAllWindows()
    #end gwpy


if __name__ == "__main__":
    main()
    print("My program took", timer() - start, "to run")
