from ctypes import cast, c_char_p
from pyglet import window
from pyglet.gl import Config, glGetString, GL_VERSION, GL_VENDOR, GL_RENDERER

def to_str(s):
    if s is None:
        return None
    try:
        return s.decode()
    except Exception:
        try:
            return cast(s, c_char_p).value.decode()
        except Exception:
            return repr(s)

try:
    cfg = Config(double_buffer=True, depth_size=24, major_version=3, minor_version=3, forward_compatible=True)
    w = window.Window(width=100, height=100, config=cfg, visible=False)
    print("Created window. Querying GL info...")
    print("GL_VERSION:", to_str(glGetString(GL_VERSION)))
    print("GL_VENDOR:", to_str(glGetString(GL_VENDOR)))
    print("GL_RENDERER:", to_str(glGetString(GL_RENDERER)))
    w.close()
except Exception as e:
    print("Core context test failed:", repr(e))
