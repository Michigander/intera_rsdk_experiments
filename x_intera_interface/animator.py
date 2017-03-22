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

from multiprocessing import Process

class Animator(object):
    """
    Animate a face.

    """
    def __init__(self, face):
        """
        Constructor

        """
        self._face = face

    def animate_sequence(sequence):
        """
        Run a list of animation groups in sequence

        """
        for batch in sequence:

            animate_series()

    def animate_series(*animations):
        """
        Run a series of animations in parallel

        """
        processes = []

        for animation in animations:
            p = Process(target=animation)
            p.start()
            processes.append(p)

        for process in processes:
            process.join()

    def toggle_eyebrow(self, side):
        """
        Raise the eyebrow on the specified side

        """

    def toggle_eye(self, side):
        """
        Close the eye on the specified side

        """
