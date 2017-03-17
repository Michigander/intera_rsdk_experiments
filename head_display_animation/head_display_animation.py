#!/usr/bin/env python

import intera_interface
import argparse
import rospy
import glob

def main():
    """ RSDK animation experiment.

    Pass the directory holding frame images.
    
    """
    epilog = """
Notes:
    Max screen resolution is 1024x600.
    Images are always aligned to the top-left corner.
    Image formats are those supported by OpenCv - LoadImage().
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__,
                                     epilog=epilog)
    required = parser.add_argument_group('required arguments')
    required.add_argument('-d','--dir', nargs='+',
                          help='Absolute path to the directory containing frame image files')
    parser.add_argument(
        '-l', '--loop', action="store_true",
        help='Display images in loop, add argument will display images in loop'
    )
    parser.add_argument(
        '-r', '--rate', type=float, default=1.0,
        help='Image display frequency for multiple and looped images.'
    )

    args = parser.parse_args()
    
    rospy.init_node("head_display_animation_example", anonymous=True)
    
    head_display = intera_interface.HeadDisplay()
    
    frames = extract_frames(args.dir)

    head_display.display_image(frames, args.loop, args.rate)
    

def extract_frames( path ):
    filenames = ""

    for filename in glob.glob( path ): 
        filenames += " "+filename
        
    print filenames
    return filenames

if __name__ == '__main__':
    main()
