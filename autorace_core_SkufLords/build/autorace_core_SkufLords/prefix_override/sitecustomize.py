import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/maxim/git_reps/Robotech/autorace_core_SkufLords/install/autorace_core_SkufLords'
