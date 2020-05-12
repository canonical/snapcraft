#! /usr/bin/env python
# encoding: utf-8
# Thomas Nagy, 2006-2012 (ita)

# the following two variables are used by the target "waf dist"
VERSION='0.0.1'
APPNAME='cc_test'

# if you want to cross-compile, use a different command line:
# CC=mingw-gcc AR=mingw-ar waf configure build

top = '.'

from waflib import Configure, Logs, Utils
#Configure.autoconfig = True # True/False/'clobber'

def options(opt):
	opt.load('compiler_c gnu_dirs')

def configure(conf):
	conf.load('compiler_c gnu_dirs')
	conf.check_cc(fragment="int main() { return 0; }\n")

	try:
		conf.check_cc(fragment="int main() { return 0; }\n", execute=True) # 1
	except conf.errors.WafError:
		Logs.warn('You are probably using a cross-compiler (disabling specific configuration tests)')
		conf.check_library(test_exec=False)
	else:
		conf.check_cc(fragment="""#include<stdio.h>\nint main(){fprintf(stderr, "mu"); printf("%d", 22);return 0;}\n""", execute=True, define_name='HAVE_MU')
		conf.check_library(test_exec=True)

	conf.check_cc(lib='m', cflags='-Wall', defines=['var=foo', 'x=y'], uselib_store='M', mandatory=False)
	conf.check_large_file(mandatory=False)
	conf.check_inline()
	conf.check_endianness()

	conf.multicheck(
		{'header_name':'stdio.h'},
		{'header_name':'unistd.h'},
		{'header_name':'stdlib.h'},
		msg       = 'Checking for standard headers',
		mandatory = False
	)
	conf.check_cc(header_name='stdio.h', auto_add_header_name=True)
	#conf.check_cc(header_name='unistd.h')
	conf.check_cc(fragment='int main() {return 0;}\n')
	conf.write_config_header('config.h')

	# exclude system libraries, force a particular folder (see strictlib below)
	#conf.check(features='c cprogram strictlib', lib = 'gif', libpath = ['/opt/lib'])

def build(bld):
	bld.env.DEFINES=['WAF=1']

	bld.recurse('program stlib shlib')
	#bld.install_files('/tmp/foo', 'wscript')
	#bld.env.PREFIX='/tmp/foo'
	bld.install_files('${PREFIX}/', 'program/a.h  program/main.c', relative_trick=False)
	bld.install_as('${PREFIX}/gnigni.txt', 'wscript')
	bld.symlink_as('${PREFIX}/libfoo.so', 'wscript')

	p = Utils.subst_vars('${PREFIX}/gnigni.txt', bld.env)
	bld.symlink_as('${PREFIX}/share/gnigni-abs.txt', p, relative_trick=False)
	bld.symlink_as('${PREFIX}/share/gnigni-rel.txt', p, relative_trick=True)

	bld.env.FOO =['m', 'ncurses']
	bld.env.ST = '-L%s'
	bld.env.A = 'aye'
	bld.env.B = 'doh'
	bld.env.SRCA = ['aaa']
	bld(rule='echo ${ST:FOO} ${ST:SRC} ${A}${B} ${ST:SRCA} ${ST:SRC[0].abspath()}',
		always=True, source='wscript', shell=1, name='Shell')
	if not Utils.is_win32:
		bld(rule='echo ${ST:FOO} ${ST:SRC} ${A}${B} ${ST:SRCA} ${ST:SRC[0].abspath()}',
		always=True, source='wscript', shell=0, cls_keyword=lambda x:'Trying again', name='NoShell')

	# illustrate how to add a command 'foo' and to execute things in it
	if bld.cmd == 'foo':
		def bar(bld):
			print('myprogram exit status is',
				bld.exec_command(bld.get_tgen_by_name('myprogram').link_task.outputs[0].abspath()))
		bld.add_post_fun(bar)
		#bld(rule='echo ${SRC} ${tsk.generator.newsize}', newsize='256x256', source='wscript')

# command examples

from waflib.Build import BuildContext
class foo_class(BuildContext):
	cmd = 'foo'

from waflib.Context import Context
class package_class(Context):
	"""just a test, try calling 'waf package' """
	cmd = 'package'
	fun = 'package'

def package(ctx):
	print('just a test', ctx.path.ant_glob('wscript'))

# and a task generator method example

from waflib import TaskGen
@TaskGen.feature('strictlib')
def check_lib_in_libpath(self):
	#For use in a configuration check: raise an exception
	#if the library file does not exist in the folders pointed by 'libpath'
	pths = self.to_list(getattr(self, 'libpath', []))
	if pths:
		for l in self.to_list(Utils.to_list(getattr(self, 'lib', []))):
			for x in pths:
				names = Utils.listdir(x)
				lname = self.env.cshlib_PATTERN % l
				if lname in names:
					break
			else:
				self.bld.fatal('wrong path for the library %s' % l)

