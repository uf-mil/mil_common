#!/usr/bin/env python
import cv2
import numpy as np
from sub8_tools import text_effects

# Image providers ==============
from image_providers import RandomColors, Webcam

image_providers = [RandomColors, Webcam] 

# Tools ====================
from tools import PaintBrushTool, PolygonTool

tools = [PaintBrushTool, PolygonTool]


'''
A main interface for various labeling tools.
'''

p = text_effects.Printer()
# General Labeler interface ===================================================

class LabelerInterface():
    '''Provides tools to create polygon segments'''
    def __init__(self, *args, **kwargs):
        blank = np.zeros(shape=(480, 640)).astype(np.uint8)
        self._draw_data = {'image': np.copy(blank), 'mask': np.copy(blank), 
                           'overlay': np.copy(blank), 'cursor': np.copy(blank)}

        self._reserved_keys = {'q': 'quit', 's': 'save and continue', ' ': 'continue',
                               'c': 'clear screen', 'b': 'back one image'}

        # Maps shortcut keys to tools
        self.tools = self._generate_hotkeys(tools)
        self.active_tool = self.tools.values()[0]
        
        # Where was the last position of the cursor 
        self.last_mouse = None

        # So we can go back an image
        self.in_back_image = False
        self.last_image = np.dstack([blank] * 3)

        self._name = "tool"
        cv2.namedWindow(self._name, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self._name, self._mouse_cb)

    def _generate_hotkeys(self, tools):
        ''' Make hotkeys for each tool. Make sure each tool class has a unique name '''
        if len(tools) == 0:
            print p.red("No tools found!\n")
            exit()

        hotkeys = {}
        for tool in tools:
            name = tool.name.lower()
            letter_index = 0
            letter = name[letter_index]
            while letter in hotkeys.keys() or \
                  letter in self._reserved_keys:
                letter = name[letter_index]
                letter_index += 1

                if letter_index >= len(name):
                    # Too many tools, generate random letter
                    name = 'abcdefghijklmnopqrstuvwxyz'
                    letter_index = 0

            hotkeys[letter] = tool(self._draw_data)

        return hotkeys
    
    def _mouse_cb(self, *args, **kwargs):
        ''' Handles drawing and what not '''
        self.last_mouse = (args[1], args[2])
        self.active_tool.mouse_cb(*args, **kwargs)

    def _apply_layers_to_image(self, layers, image):
        _disp_img = np.copy(image).astype(np.uint32)
        for layer in layers:
            if len(layer.shape) == 2:
                # Meaning it's a one channel image
                layer = np.dstack((layer, layer, layer))

            _disp_img += layer.astype(np.uint32)

        _disp_img = np.clip(_disp_img, 0, 255).astype(np.uint8)
        return _disp_img

    def _print_options(self):
        print p.text("\nThe current active tool is the ").bold(self.active_tool.name).text(":")

        # Template line
        line = p.bold('\t[ {} ] : ').text("{}")

        print p.text("\nGeneral controls are as follows:")
        for key, command in self._reserved_keys.items():
            print line.format(key, command)

        print p.text("\nAvalible Tools:")
        for key, tool in self.tools.items():
            print line.format(key, tool.name) 

        self.active_tool.print_options()

    def start_labeling(self, image_provider):
        self._print_options()

        key = None

        mask_opacity = 0.3
        overlay_opacity = 0.9
        cursor_opacity = 1.0
        while key is not 'q':
            if self.in_back_image:
                # We are currently editing the previous image
                self.active_tool.image = self.last_image
                self.in_back_image = False
            else:
                self.active_tool.image = image_provider.get_next_image()
             
            # We stay on this image until a reserved_key gets pressed
            key = None
            while key not in self._reserved_keys:
                layers = [self.active_tool.mask * mask_opacity, 
                          self.active_tool.overlay * overlay_opacity,
                          self.active_tool.cursor * cursor_opacity]
                          
                cv2.imshow(self._name, self._apply_layers_to_image(layers, self.active_tool.image)) 

                key = chr(cv2.waitKey(10) & 0xFF)
                
                if key in self.tools.keys():
                    self.active_tool = self.tools[key]
                    self.active_tool.set_active(self.last_mouse)
                    self._print_options()

                elif key not in self._reserved_keys:
                    self.active_tool.key_press(key)

                elif key == 's':
                    # save
                    self.save()

                elif key == 'b':
                    # go back an image
                    key = None
                    if not self.in_back_image:
                        current_image = self.active_tool.image
                        self.active_tool.image = self.last_image
                        self.last_image = current_image

                    self.in_back_image = True

                elif key == 'c':
                    # clear
                    key = None
                    self.active_tool.set_active(self.last_mouse)
                    self.active_tool.clear_mask()

            if not self.in_back_image:
                self.last_image = self.active_tool.image

        cv2.destroyAllWindows()
        choice = raw_input(p.text("\nDo you want to save first? ").bold("(y/N) : "))
        if choice.lower() == 'y':
            self.save()

    def save(self):
        print "SAVING pair"

# =============================================================================

def _menu_options(options, name=None):
    ''' Returns the object associated with user selected entry
        options := list of objects with a 'name' field
    '''
    title = p.text("Please select from the following")
    title += ":" if name is None else p.bold(" {}:".format(name))
    print title

    numbers = []
    index = 1
    for class_type in options:
        _number = p.bold("\t[ {} ] :".format(index))
        _option = p.text(class_type.name)

        print _number, _option
        numbers.append(class_type)
        index += 1
    
    while True:
        try:
            # All options are displayed, wait for user input
            selection = int(raw_input("\nSelect from the above options: "))
            res = numbers[selection - 1]

        except KeyboardInterrupt:
            print
            exit()

        except:
            # There was some kind of error in the input
            print p.red("\nPlease enter a number between 1 and {}".format(len(options)))

        else:
            # Valid input
            break

    print "=" * 50
    return res()

if __name__ == "__main__":

    print p.bold("\nWelcome to the image segmentation tool!")

    image_provider = _menu_options(image_providers, "image providers")
    image_provider.load_source()
    labeler = LabelerInterface()
    labeler.start_labeling(image_provider)


