File Edit Options Buffers Tools Python Help                                                            import rospy

from sensor_msgs.msg import Image


class SimpleHeadDisplay(object):
    
    def __init__(self):
        """                                                                                             
        Constructor                                                                                     
        """
        self._image_pub = rospy.Publisher('/robot/head_display', Image, latch=True, queue_size=10)


    def display_image(self, img_msg):
        """                                                                                             
        Displays image(s) to robot's head                                                               
                                                                                                        
        @type image_path: list                                                                          
        @param image_path: the relative or absolute file path to the image file(s)                      
                                                                                                        
        @type display_in_loop: bool                                                                     
        @param display_in_loop: Set True to loop through all image files infinitely                     
                                                                                                        
        @type display_rate: float                                                                       
        @param display_rate: the rate to publish image into head                                        
        """
        rospy.logdebug("Display images in loop:'{0}', frequency: '{1}'".format(display_in_loop, display\
_rate))

        image_msg = []
        image_list = image_path if isinstance(image_path, list) else [image_path]
        for one_path in image_list:
            cv_img = self._setup_image(one_path)
            if cv_img:
                image_msg.append(cv_img)

        if not image_msg:
            rospy.logerr("Image message list is empty")
        else:
            r = rospy.Rate(display_rate)
            while not rospy.is_shutdown():
                for one_msg in image_msg:
                    self._image_pub.publish(one_msg)
                    r.sleep()
                if not display_in_loop:
                    break

