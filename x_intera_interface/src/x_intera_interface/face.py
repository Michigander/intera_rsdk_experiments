# Copyright (c) 2017 GFIII X RR
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import x_intera_interface as xii
import skimage as ski
import numpy as np
import cv_bridge
import cv2

class Face(object):
    """
        Experimental interface class for the face of an Intera Robot.

        Used to control facial expressions.
    """
    def __init__(self):
        """
        Constructor

        """
        self._face_img_file = xii.FACE_IMG_FILE
        self._face_img_

        # shared numpy
        self._face_img = np.fromfile(x_intera_interface.FACE_IMAGE_FILE)

    def get_face_img(self):
        """
        Return the current face.

        """
        return self._face_img
