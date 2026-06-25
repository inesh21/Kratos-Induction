import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/harinesh21/Harinesh/induction_ws/install/turtle_controller'
