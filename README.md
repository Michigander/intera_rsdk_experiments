# Intera RSDK Experiments
#

>>> Experiments in applying and extending the Software Developer Kit for Sawyer, a collaborative robot by Rethink Robotics.

### x_intera_interface
This experimental interface is composed of classes providing programming interfaces for various components of the robot. Some components are pure - such as the **gripper**, **head_display**, and **camera** - while others like **face** are new abstractions.

1. **simple_head_display.py** : A pure components class. provides a method for sending a single image to the display screen. Differs from the native sdk's head_display by not requiring an image to be read from file.

2. **face.py** : This abstracted class wraps the head_display component class and provides methods for running animations of a robot face.
