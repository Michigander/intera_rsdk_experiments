#!/usr/bin/env python

import intera_interface
import argparse
import rospy

import os
from os import listdir
from os.path import abspath, isfile, join

from PIL import Image 

# head display info
HEAD_IMAGE_WIDTH = 1024
HEAD_IMAGE_HEIGHT = 600 
HEAD_IMAGE_FORMAT = 'JPEG' 

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

    required.add_argument(
        '-f','--file', 
        help='Absolute path to the directory containing frame image files')
    parser.add_argument(
        '-l', '--loop', action="store_true",
        help='Display images in loop, add argument will display images in loop')
    parser.add_argument(
        '-r', '--rate', type=float, default=1.0,
        help='Image display frequency for multiple and looped images.')

    args = parser.parse_args()
    
    rospy.init_node("head_display_animation_example", anonymous=True)
    
    head_display = intera_interface.HeadDisplay()

    frames_string = process_image(args.file)

    head_display.display_image(frames_string, args.loop, args.rate)    

def analyse_image(path):
    '''                                                                                                                                                        
    Pre-process pass over the image to determine the mode (full or additive).                                                                                  
    Necessary as assessing single frames isn't reliable. Need to know the mode                                                                                 
    before processing all frames.                                                                                                                              
    '''
    im = Image.open(path)
    results = {
        'size': im.size,
        'mode': 'full',
    }
    try:
        while True:
            if im.tile:
                tile = im.tile[0]
                update_region = tile[1]
                update_region_dimensions = update_region[2:]
                if update_region_dimensions != im.size:
                    results['mode'] = 'partial'
                    break
            im.seek(im.tell() + 1)
    except EOFError:
        pass
    return results

def process_image(path):

    '''                                                                                                                                                        
    Iterate the GIF, extracting each frame.                                                                                                                    
    '''
    mode = analyse_image(path)['mode']

    im = Image.open(path)
    
    i = 0
    p = im.getpalette()
    last_frame = im.convert('RGBA')
    
    frame_filenames = []

    try:
        while True:
            print "saving %s (%s) frame %d, %s %s" % (path, mode, i, im.size, im.tile)

            '''                                                                                                                                                
            If the GIF uses local colour tables, each frame will have its own palette.                                                                         
            If not, we need to apply the global palette to the new frame.                                                                                      
            '''
            if not im.getpalette():
                im.putpalette(p)

            new_frame = Image.new('RGBA', im.size)

            '''                                                                                                                                                
            Is this file a "partial"-mode GIF where frames update a region of a different size to the entire image?                                            
            If so, we need to construct the new frame by pasting it on top of the preceding frames.                                                            
            '''
            if mode == 'partial':
                new_frame.paste(last_frame)

            new_frame.paste(im, (0,0), im.convert('RGBA'))
            
            # build filename
            filename = '%s%s-%d.jpg' % ('/data/users/gfinnie/Documents/media/tmp/',''.join(os.path.basename(path).split('.')[:-1]),i)
            frame_filenames.append(filename)
            
            # resize frame to fit display 
            resized_frame = new_frame.resize((HEAD_IMAGE_WIDTH,HEAD_IMAGE_HEIGHT),Image.ANTIALIAS)
            resized_frame.save(filename, HEAD_IMAGE_FORMAT)
            
            # move to next frame
            i += 1
            last_frame = new_frame
            im.seek(im.tell() + 1)
   
    except EOFError:
        pass
    
    return frame_filenames

if __name__ == '__main__':
    main()
