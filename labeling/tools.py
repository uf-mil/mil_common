import numpy as np
import cv2

from sub8_tools import text_effects


p = text_effects.Printer()

class BaseTool(object):
    '''Used by the LabelerInterface to eaisly switch between different tools.
    '''
    name = "general tool"

    def init(self):
        ''' Set up any variables needed here '''
        pass

    def key_press(self, key):
        ''' Called when a non reserved key is pressed '''
        pass

    def print_options(self):
        ''' Print data like hotkeys or info '''
        pass

    def mouse_cb(self, event, x, y, flags, param):
        ''' cv2 callback to be called for this tool when it is active '''
        pass

    def set_active(self, cursor_pos=None):
        ''' 
        Method called when this tool is now the active one 
            cursor_pos := the (x, y) position of the cursor
        '''
        self.clear_overlay()
        self.clear_cursor()

    def clear_mask(self):
        ''' Clear the current mask '''
        self.mask = np.zeros(shape=self.image.shape[:2])

    def clear_overlay(self):
        ''' Clear the current overlay '''
        self.overlay = np.zeros(shape=self.image.shape)

    def clear_cursor(self):
        ''' Clear the current cursor '''
        self.cursor = np.zeros(shape=self.image.shape)

    def __init__(self, draw_data):
        ''' Use self.init instead of this function '''
        # Stores data used by this class. Do not interact directly with or things could break
        self._draw_data = draw_data
        self.init()

    @property
    def image(self):
        ''' Gets the current image being segmeneted '''
        return self._draw_data['image']

    @image.setter
    def image(self, image):
        ''' Sets the current image being segmeneted '''
        self._draw_data['image'] = image

    @property
    def mask(self):
        ''' Gets the current mask for this segmentation '''
        return self._draw_data['mask']

    @mask.setter
    def mask(self, mask):
        ''' Sets the current mask for this segmentation '''
        self._draw_data['mask'] = mask

    @property
    def overlay(self):
        ''' Gets the current overlay for this segmentation '''
        return self._draw_data['overlay']
    
    @overlay.setter
    def overlay(self, overlay):
        ''' Sets the current overlay for this segmentation '''
        self._draw_data['overlay'] = overlay

    @property
    def cursor(self):
        ''' Gets the current cursor for this segmentation '''
        return self._draw_data['cursor']
    
    @cursor.setter
    def cursor(self, cursor):
        ''' Sets the current cursor for this segmentation '''
        self._draw_data['cursor'] = cursor


class PolygonTool(BaseTool):
    name = "Polygon Tool"

    def init(self):
        self.lines = []
        self._line_width = 2
        self._line_color = (255, 255, 255)
        self._cursor_width = 2
        self._cursor_color = (100, 150, 255)
        self.in_draw = False

    def print_options(self):
        print p.text("\nClick to draw polygons.")
        print p.text("Hotkeys are as follows:")
        print p.bold("\t[ - ] : ").text("delete the last node")
        print p.bold("\t[ = ] : ").text("complete the polygon")

    def set_active(self, cursor_pos=None):
        self.clear_cursor()
        self.clear_overlay()
        self.lines = []

        if cursor_pos is not None:
            self._last_x, self._last_y = cursor_pos
            self._draw_cursor()

    def key_press(self, key):
        if len(self.lines) < 1:
            return

        if key == '-':
            self.in_draw = True
            del self.lines[-1]

        if key == '=':
            self.in_draw = False

            pts = np.array(self.lines).reshape((-1, 1, 2)).astype(np.int32)
            cv2.fillPoly(self.mask, [pts], 255)

            self.lines.append(self.lines[0])
            self.clear_cursor()
        
        if key in ['-', '=']:
            self._redraw()

    def _redraw(self):
        self.overlay = np.zeros(shape=self.image.shape)
        pts = np.array(self.lines).reshape((-1, 1, 2)).astype(np.int32)
        cv2.polylines(self.overlay, [pts], False, self._line_color, self._line_width)

        # for i in range(len(self.lines) - 1):
        #     cv2.line(self.overlay, self.lines[i], self.lines[i + 1], 
        #              self._line_color, self._line_width)

    def mouse_cb(self, event, x, y, flags, param, **kwargs):
        self._last_x = x
        self._last_y = y

        if event == cv2.EVENT_LBUTTONDOWN:
            if self.in_draw:
                cv2.line(self.overlay, (x, y), self.lines[-1], 
                         self._line_color, self._line_width)
            else:
                self.lines = []
                self.clear_overlay()
                self.in_draw = True

            self.lines.append((x, y))

        self._draw_cursor()

    def _draw_cursor(self):
        x = self._last_x
        y = self._last_y
        h_low = (x - 10, y)
        h_high = (x + 10, y)
        v_low = (x, y - 10)
        v_high = (x, y + 10)

        self.clear_cursor()
        cv2.line(self.cursor, h_low, h_high, self._cursor_color, self._cursor_width)
        cv2.line(self.cursor, v_low, v_high, self._cursor_color, self._cursor_width)

        if self.in_draw:
            cv2.line(self.cursor, (x, y), self.lines[-1], self._line_color, self._line_width)


class PaintBrushTool(BaseTool):
    name = "Paint Brush Tool"

    def init(self):
        self.size = 10
        self.last_x = -1
        self.last_y = -1

        self._cursor_color = (255, 255, 255)

    def print_options(self):
        print p.text("\nClick and drag to draw. Shift click and drag to erase.")
        print p.text("Hotkeys are as follows:")
        print p.bold("\t[ , ] : ").text("decrease cursor size.")
        print p.bold("\t[ . ] : ").text("increase cursor size.")

    def set_active(self, mouse_pos=None):
        self.clear_overlay()
        self.clear_cursor()

        if mouse_pos is not None:
            cv2.circle(self.cursor, mouse_pos, self.size, self._cursor_color, 2)

    def key_press(self, key):
        if key == ',':
            self.size -= self.size ** 0.5
        if key == '.':
            self.size += self.size ** 0.5
        
        # Force a redraw
        if key in [',', '.']:
            self.size = int(np.clip(self.size, 1, np.inf))
            self.clear_cursor()
            cv2.circle(self.cursor, (self.last_x, self.last_y), self.size, self._cursor_color, 2)

    def mouse_cb(self, event, x, y, flags, param, **kwargs):
        self.last_x = x
        self.last_y = y
        self.clear_cursor()
        cv2.circle(self.cursor, (x, y), self.size, self._cursor_color, 2)
        
        if flags == cv2.EVENT_FLAG_LBUTTON:
            cv2.circle(self.mask, (x, y), self.size, 255, -1)
            
        if flags == cv2.EVENT_FLAG_SHIFTKEY + cv2.EVENT_FLAG_LBUTTON:
            cv2.circle(self.mask, (x, y), self.size, 0, -1)
            

class MoveTool(BaseTool):
    name = "Move Tool"

    def init(self):
        self.last_x = -1
        self.last_y = -1

        self.moving = False
        self.shift_mode = False
        self.temp_mask = np.copy(self.mask)
        self.last_offset = (0, 0)
        self.dx = self.dy = 0
        
        self._line_color = (255, 255, 255)

    def print_options(self):
        desc = p.text("\nClick and drag to move the current mask. ")
        print desc.text("Shift click and drag to move with more percision")
        print p.text("Hotkeys are as follows:")
        print p.bold("\t[ - ] : ").text("undo the previous change.")

    def set_active(self, mouse_pos=None):
        self.clear_overlay()
        self.clear_cursor()

        self.shift_mode = False
        self.moving = False
        self.temp_mask = np.copy(self.mask)
        self.last_offset = (0, 0)
        self.dx = self.dy = 0

    def key_press(self, key):
        # R, S, T, Q <= are the letters for the arrow keys
        if key == 'R':
            # move up
            pass
        elif key == 'S':
            # move right
            pass
        elif key == 'T':
            # move down
            pass
        elif key == 'Q':
            # move right
            pass
        
        elif key == '-':
            # Undoes the most recent move
            t = np.array([[1, 0, 0], [0, 1, 0]])
            t[:, 2] -=  self.last_offset
            self.mask = cv2.warpAffine(self.mask, t.astype(np.float32), self.mask.shape[::-1])

    def get_transform(self, x, y):
        mod = 1
        if self.shift_mode:
            mod = 0.3

        dx = (x - self.last_x) * mod
        dy = (y - self.last_y) * mod

        self.dx += dx
        self.dy += dy
        return np.array([[1, 0, self.dx], [0, 1, self.dy]]).astype(np.float64)

    def mouse_cb(self, event, x, y, flags, param, **kwargs):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.moving = True
            self.temp_mask = np.copy(self.mask)
            self.last_offset = (0, 0)

        elif event == cv2.EVENT_LBUTTONUP:
            self.moving = False
            self.shift_mode = False
            self.last_offset = (self.dx, self.dy)
            self.dx = self.dy = 0
        
        self.shift_mode = flags == cv2.EVENT_FLAG_SHIFTKEY + cv2.EVENT_FLAG_LBUTTON


        self.clear_overlay()
        if self.moving:
           t = self.get_transform(x, y)
           self.mask = cv2.warpAffine(self.temp_mask, t, self.mask.shape[::-1])

        self.last_x = x
        self.last_y = y
        
