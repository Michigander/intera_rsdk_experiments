#!/usr/bin/env python
# Copyright (c) 2017 gmf X rethink
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
import intera_interface as ii
import x_intera_interface as xii

test_sequences = { 'Sequence 1' :  }

def main(): 
    print '[*] Building face ...'
    face = xii.Face()
    print '[*] Done.'

    print '[*] Building animator ...'
    animator = xii.Animator()
    print '[*] Done.'

    print '[*] Building face display ...'
    display = xii.FaceDisplay()
    print '[*] Done.'

    print '[*] Displaying face ...'
    display.display_face()
    print '[*] Done.'

    print '[*]- Running single animations ...'
    run_single_tests()
    print '[*]- Done.'

    print '[*]- Running sequence animations ...'
    run_sequence_tests()
    print '[*]- Done.'

    print '[*]- Running series animations ...'
    run_series_tests()
    print '[*]- Done.'

def run_single_tests():

    print '\t[1] Toggling right eyebrow ...'
    animator.animate(toggle_right_eyebrow)
    print '\t[*] Done.'

    print '\t[2] Toggling left eyebrow ...'
    animator.animate(toggle_left_eyebrow)
    print '\t[*] Done.'

    print '\t[3] Toggling left eye ...'
    animator.animate(toggle_left_eye)
    print '\t[*] Done.'

    print '\t[4] Toggling right eye ...'
    animator.animate(toggle_right_eye)
    print '\t[*] Done.'

def run_sequence_tests():

    for title, test in test_sequences:
        print '\t[*] Running sequence ' +
        animator.animate_sequence(test_sequence)

def run_series_tests():
if __name__ == '__main__':
    main()
