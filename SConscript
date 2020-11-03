from building import *
Import('rtconfig')

src   = []
cwd   = GetCurrentDir()

# add g1632 src files.
if GetDepend('PKG_USING_G1632'):
    src += Glob('g1632.c')

if GetDepend('PKG_USING_G1632_EXAMPLE'):
    src += Glob('g1632_example.c')

# add g1632 include path.
path  = [cwd]

# add src and include to group.
group = DefineGroup('g1632', src, depend = ['PKG_USING_G1632'], CPPPATH = path)

Return('group')
