# Copyright (c) 2012-2013 ARM Limited
# All rights reserved.
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
#
# Copyright (c) 2006-2008 The Regents of The University of Michigan
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Authors: Steve Reinhardt

# Simple test script
#
# "m5 test.py"

from __future__ import print_function

import optparse
import sys
import os

import m5
from m5.defines import buildEnv
from m5.objects import *
from m5.util import addToPath, fatal, warn

addToPath('../')

from ruby import Ruby

from common import Options
from common import Simulation
from common import CacheConfig
from common import CpuConfig
from common import MemConfig
from common.Caches import *
from common.cpu2000 import *

# Check if KVM support has been enabled, we might need to do VM
# configuration if that's the case.
have_kvm_support = 'BaseKvmCPU' in globals()
def is_kvm_cpu(cpu_class):
    return have_kvm_support and cpu_class != None and \
        issubclass(cpu_class, BaseKvmCPU)

def get_processes(options):
    """Interprets provided options and returns a list of processes"""

    multiprocesses = []
    inputs = []
    outputs = []
    errouts = []
    pargs = []

    workloads = options.cmd.split(';')
    if options.input != "":
        inputs = options.input.split(';')
    if options.output != "":
        outputs = options.output.split(';')
    if options.errout != "":
        errouts = options.errout.split(';')
    if options.options != "":
        pargs = options.options.split(';')

    idx = 0
    for wrkld in workloads:
        process = Process(pid = 100 + idx)
        process.executable = wrkld
        process.cwd = os.getcwd()

        if options.env:
            with open(options.env, 'r') as f:
                process.env = [line.rstrip() for line in f]

        if len(pargs) > idx:
            process.cmd = [wrkld] + pargs[idx].split()
        else:
            process.cmd = [wrkld]

        if len(inputs) > idx:
            process.input = inputs[idx]
        if len(outputs) > idx:
            process.output = outputs[idx]
        if len(errouts) > idx:
            process.errout = errouts[idx]

        multiprocesses.append(process)
        idx += 1

    if options.smt:
        assert(options.cpu_type == "DerivO3CPU")
        return multiprocesses, idx
    else:
        return multiprocesses, 1


parser = optparse.OptionParser()
Options.addCommonOptions(parser)
Options.addSEOptions(parser)

if '--ruby' in sys.argv:
    Ruby.define_options(parser)

(options, args) = parser.parse_args()

if args:
    print("Error: script doesn't take any positional arguments")
    sys.exit(1)
##########################################################################################
##
## set the SPEC2006 in the multiprocess[]
##
##########################################################################################
multiprocesses = []
binary_dir = '/home/ar443/jia/gem5/cpu2006/benchspec/CPU2006/'
data_dir = '/home/ar443/jia/gem5/cpu2006/benchspec/CPU2006/'
# idx = 0
#====================

 #202000522: Cache sensitive

#====================

#482.sphinx3
sphinx3=Process()
sphinx3.executable =  binary_dir+'482.sphinx3/exe/sphinx_livepretend_base.amd64-m64-gcc41-nn'
sphinx3.cmd = [sphinx3.executable]+['ctlfile', '.', 'args.an4']
sphinx3.output = 'an4.out'
 
#483.xalancbmk
xalancbmk=Process()
xalancbmk.executable =  binary_dir+'483.xalancbmk/exe/Xalan_base.amd64-m64-gcc41-nn'
xalancbmk.cmd = [xalancbmk.executable]+['-v','test.xml','xalanc.xsl']
xalancbmk.output = 'test.out'



#====================

 #new

#====================



#-----------------------------------------8 applications of SPEC CPU2006------------------------------------------------
#403.gcc
gcc = Process(pid = 100)
gcc.executable = '/home/ar443/jia/gem5/cpu2006/benchspec/CPU2006/403.gcc/exe/gcc_base.amd64-m64-gcc41-nn'
data = '/home/ar443/jia/gem5/cpu2006/benchspec/CPU2006/403.gcc/data/ref/input/166.i'
output='/home/ar443/jia/gem5/cpu2006/benchspec/CPU2006/403.gcc/data/ref/output/166.s'
gcc.cmd = [gcc.executable] + [data]+['-o',output]
gcc.output = '/home/ar443/jia/gem5/cpu2006/benchspec/CPU2006/403.gcc/data/ref/output/166.out'
gcc.Process_DSid = 0

#401.bzip2
bzip2 = Process(pid = 101)
bzip2.executable = '/home/ar443/jia/gem5/cpu2006/benchspec/CPU2006/401.bzip2/exe/bzip2_base.amd64-m64-gcc41-nn'
data='/home/ar443/jia/gem5/cpu2006/benchspec/CPU2006/401.bzip2/data/all/input/input.combined'
bzip2.cmd = [bzip2.executable] + [data] + ['200']
bzip2.output = '/home/ar443/jia/gem5/cpu2006/benchspec/CPU2006/401.bzip2/data/ref/output/input.combined.out'


#470.lbm
lbm=Process(pid = 102)
lbm.executable =  '/home/ar443/jia/gem5/cpu2006/benchspec/CPU2006/470.lbm/exe/lbm_base.amd64-m64-gcc41-nn'
data = '/home/ar443/jia/gem5/cpu2006/benchspec/CPU2006/470.lbm/data/ref/input/100_100_130_ldc.of'
lbm.cmd = [lbm.executable]+['3000', 'reference.dat', '0', '0' ,data]
lbm.output = '/home/ar443/jia/gem5/cpu2006/benchspec/CPU2006/470.lbm/data/ref/output/lbm.out'

#437.leslie3d
leslie3d=Process(pid = 103)
leslie3d.executable = '/home/ar443/jia/gem5/cpu2006/benchspec/CPU2006/437.leslie3d/exe/leslie3d_base.amd64-m64-gcc41-nn'
stdin = '/home/ar443/jia/gem5/cpu2006/benchspec/CPU2006/437.leslie3d/data/ref/input/leslie3d.in'
leslie3d.cmd = [leslie3d.executable]
leslie3d.input=stdin
leslie3d.output='/home/ar443/jia/gem5/cpu2006/benchspec/CPU2006/437.leslie3d/data/ref/output/leslie3d.stdout'
 
#462.libquantum
libquantum=Process(pid = 104)
libquantum.executable = '/home/ar443/jia/gem5/cpu2006/benchspec/CPU2006/462.libquantum/exe/libquantum_base.amd64-m64-gcc41-nn'
libquantum.cmd = [libquantum.executable]+['1397','8']
libquantum.output = '/home/ar443/jia/gem5/cpu2006/benchspec/CPU2006/462.libquantum/data/ref/output/ref.out'


#433.milc
milc=Process(pid = 105)
milc.executable = '/home/ar443/jia/gem5/cpu2006/benchspec/CPU2006/433.milc/exe/milc_base.amd64-m64-gcc41-nn'
stdin='/home/ar443/jia/gem5/cpu2006/benchspec/CPU2006/433.milc/data/ref/input/su3imp.in'
milc.cmd = [milc.executable]
milc.cwd = '/home/ar443/jia/gem5/cpu2006/benchspec/CPU2006/433.milc/data/ref/input/'
milc.input=stdin
milc.output='/home/ar443/jia/gem5/cpu2006/benchspec/CPU2006/433.milc/data/ref/output/su3imp.out'


#471.omnetpp  
omnetpp=Process(pid = 106)
omnetpp.executable =  binary_dir+'471.omnetpp/exe/omnetpp_base.amd64-m64-gcc41-nn'
data=data_dir+'471.omnetpp/data/test/input/omnetpp.ini'
omnetpp.cmd = [omnetpp.executable]+[data]     # remove -m3500
omnetpp.output = 'omnetpp.log'

#456.hmmer
hmmer=Process(pid = 107)
hmmer.executable =  binary_dir+'456.hmmer/exe/hmmer_base.amd64-m64-gcc41-nn'
data=data_dir+'456.hmmer/data/test/input/bombesin.hmm'
hmmer.cmd = [hmmer.executable]+['--fixed', '0', '--mean', '325', '--num', '5000', '--sd', '200', '--seed', '0', data]
hmmer.output = 'bombesin.out'

#450.soplex
soplex=Process(pid = 108)
soplex.executable = '/home/ar443/jia/gem5/cpu2006/benchspec/CPU2006/450.soplex/exe/soplex_base.amd64-m64-gcc41-nn'
data = '/home/ar443/jia/gem5/cpu2006/benchspec/CPU2006/450.soplex/data/ref/input/ref.mps'
soplex.cmd = [soplex.executable]+['-m3500',data]
soplex.output = '/home/ar443/jia/gem5/cpu2006/benchspec/CPU2006/450.soplex/data/ref/output/ref.out'
# 

###############################################//// replacement ////###############################################################
#453.povray
povray=Process(pid = 109)
povray.executable =  binary_dir+'453.povray/exe/povray_base.amd64-m64-gcc41-nn'
data=data_dir+'453.povray/data/test/input/SPEC-benchmark-test.ini'
#povray.cmd = [povray.executable]+['SPEC-benchmark-test.ini']
povray.cmd = [povray.executable]+[data]
povray.output = 'SPEC-benchmark-test.stdout'
 
#454.calculix
calculix=Process(pid = 110)
calculix.executable =  binary_dir+'454.calculix/exe/calculix_base.amd64-m64-gcc41-nn'
data=data_dir+'454.calculix/data/test/input/beampic'
calculix.cmd = [calculix.executable]+['-i',data]
calculix.output = 'beampic.log'

#444.namd
namd = Process(pid = 111)
namd.executable = '/home/ar443/jia/gem5/cpu2006/benchspec/CPU2006/444.namd/exe/namd_base.amd64-m64-gcc41-nn'
data='/home/ar443/jia/gem5/cpu2006/benchspec/CPU2006/444.namd/data/all/input/namd.input'
namd.cmd = [namd.executable] + ['--input',data,'--iterations','38','--output','/home/ar443/jia/gem5/cpu2006/benchspec/CPU2006/444.namd/data/ref/output/namd.out']
namd.output = "/home/ar443/jia/gem5/cpu2006/benchspec/CPU2006/444.namd/data/ref/output/namd.stdout"
 

#435.gromacs
gromacs = Process(pid = 112)
gromacs.executable = '/home/ar443/jia/gem5/cpu2006/benchspec/CPU2006/435.gromacs/exe/gromacs_base.amd64-m64-gcc41-nn'
data='/home/ar443/jia/gem5/cpu2006/benchspec/CPU2006/435.gromacs/data/ref/input/gromacs'
gromacs.cmd = [gromacs.executable] + ['-silent','-deffnm',data,'-nice','0']
gromacs.cwd = "/home/ar443/jia/gem5/cpu2006/benchspec/CPU2006/435.gromacs/data/ref/input/"


#------------------------------------------------------------------------------------------

multiprocesses.append(gcc)
multiprocesses.append(bzip2)   #shortcut
multiprocesses.append(lbm) 

multiprocesses.append(leslie3d)   #shortcut

multiprocesses.append(libquantum)
# multiprocesses.append(milc)       #shortcut
# 
# multiprocesses.append(omnetpp)

multiprocesses.append(hmmer)
# multiprocesses.append(soplex)
# # # # 
multiprocesses.append(calculix)  #shortcut

# multiprocesses.append(namd)      #shortcut
multiprocesses.append(gromacs)   #shortcut

#--------------------------------------new applications------------------------------------


# multiprocesses.append(omnetpp)


##########################################################################################


# multiprocesses = []
numThreads = 1

if options.bench:
    apps = options.bench.split("-")
    if len(apps) != options.num_cpus:
        print("number of benchmarks not equal to set num_cpus!")
        sys.exit(1)

    for app in apps:
        try:
            if buildEnv['TARGET_ISA'] == 'alpha':
                exec("workload = %s('alpha', 'tru64', '%s')" % (
                        app, options.spec_input))
            elif buildEnv['TARGET_ISA'] == 'arm':
                exec("workload = %s('arm_%s', 'linux', '%s')" % (
                        app, options.arm_iset, options.spec_input))
            else:
                exec("workload = %s(buildEnv['TARGET_ISA', 'linux', '%s')" % (
                        app, options.spec_input))
            multiprocesses.append(workload.makeProcess())
        except:
            print("Unable to find workload for %s: %s" %
                  (buildEnv['TARGET_ISA'], app),
                  file=sys.stderr)
            sys.exit(1)
# elif options.cmd:
#     multiprocesses, numThreads = get_processes(options)
# else:
#     print("No workload specified. Exiting!\n", file=sys.stderr)
#     sys.exit(1)


(CPUClass, test_mem_mode, FutureClass) = Simulation.setCPUClass(options)
CPUClass.numThreads = numThreads

# Check -- do not allow SMT with multiple CPUs
if options.smt and options.num_cpus > 1:
    fatal("You cannot use SMT with multiple CPUs!")

np = options.num_cpus
##########################################################################################
##
#jia=[[0,0],[1,1],[2,2],[3,2],[4,2],[5,1]]
jia=[[0,0],[1,1],[2,2],[3,3],[4,4],[5,5],[6,6],[7,7]]


##   
##   cpu = [CPUClass(cpu_id=i, itb=X86TLB(DSid=j), dtb=X86TLB(DSid=j)) for i,j in jia ]
##
##########################################################################################
system = System(#cpu = [CPUClass(cpu_id=i) for i in xrange(np)],
                cpu = [CPUClass(cpu_id=i, itb=X86TLB(DSid=i), dtb=X86TLB(DSid=i)) for i in xrange(np) ],
                # cpu = [CPUClass(cpu_id=i, itb=X86TLB(DSid=j), dtb=X86TLB(DSid=j)) for i,j in jia ],
                mem_mode = test_mem_mode,
                mem_ranges = [AddrRange(options.mem_size)],
                cache_line_size = options.cacheline_size)

if numThreads > 1:
    system.multi_thread = True

# Create a top-level voltage domain
system.voltage_domain = VoltageDomain(voltage = options.sys_voltage)

# Create a source clock for the system and set the clock period
system.clk_domain = SrcClockDomain(clock =  options.sys_clock,
                                   voltage_domain = system.voltage_domain)

# Create a CPU voltage domain
system.cpu_voltage_domain = VoltageDomain()

# Create a separate clock domain for the CPUs
system.cpu_clk_domain = SrcClockDomain(clock = options.cpu_clock,
                                       voltage_domain =
                                       system.cpu_voltage_domain)

# If elastic tracing is enabled, then configure the cpu and attach the elastic
# trace probe
if options.elastic_trace_en:
    CpuConfig.config_etrace(CPUClass, system.cpu, options)

# All cpus belong to a common cpu_clk_domain, therefore running at a common
# frequency.
for cpu in system.cpu:
    cpu.clk_domain = system.cpu_clk_domain

if is_kvm_cpu(CPUClass) or is_kvm_cpu(FutureClass):
    if buildEnv['TARGET_ISA'] == 'x86':
        system.kvm_vm = KvmVM()
        for process in multiprocesses:
            process.useArchPT = True
            process.kvmInSE = True
    else:
        fatal("KvmCPU can only be used in SE mode with x86")

# Sanity check
if options.fastmem:
    if CPUClass != AtomicSimpleCPU:
        fatal("Fastmem can only be used with atomic CPU!")
    if (options.caches or options.l2cache):
        fatal("You cannot use fastmem in combination with caches!")

if options.simpoint_profile:
    if not options.fastmem:
        # Atomic CPU checked with fastmem option already
        fatal("SimPoint generation should be done with atomic cpu and fastmem")
    if np > 1:
        fatal("SimPoint generation not supported with more than one CPUs")

for i in xrange(np):
    if options.smt:
        system.cpu[i].workload = multiprocesses
    elif len(multiprocesses) == 1:
        system.cpu[i].workload = multiprocesses[0]
    else:
        system.cpu[i].workload = multiprocesses[i]

    if options.fastmem:
        system.cpu[i].fastmem = True

    if options.simpoint_profile:
        system.cpu[i].addSimPointProbe(options.simpoint_interval)

    if options.checker:
        system.cpu[i].addCheckerCpu()

    system.cpu[i].createThreads()

if options.ruby:
    Ruby.create_system(options, False, system)
    assert(options.num_cpus == len(system.ruby._cpu_ports))

    system.ruby.clk_domain = SrcClockDomain(clock = options.ruby_clock,
                                        voltage_domain = system.voltage_domain)
    for i in xrange(np):
        ruby_port = system.ruby._cpu_ports[i]

        # Create the interrupt controller and connect its ports to Ruby
        # Note that the interrupt controller is always present but only
        # in x86 does it have message ports that need to be connected
        system.cpu[i].createInterruptController()

        # Connect the cpu's cache ports to Ruby
        system.cpu[i].icache_port = ruby_port.slave
        system.cpu[i].dcache_port = ruby_port.slave
        if buildEnv['TARGET_ISA'] == 'x86':
            system.cpu[i].interrupts[0].pio = ruby_port.master
            system.cpu[i].interrupts[0].int_master = ruby_port.slave
            system.cpu[i].interrupts[0].int_slave = ruby_port.master
            system.cpu[i].itb.walker.port = ruby_port.slave
            system.cpu[i].dtb.walker.port = ruby_port.slave
else:
    MemClass = Simulation.setMemClass(options)
    system.membus = SystemXBar()
    system.system_port = system.membus.slave
    CacheConfig.config_cache(options, system)
    MemConfig.config_mem(options, system)

    ## @jia
    # system.drambuffer = SimpleCache()
    # system.drambus = SystemXBar()
    # system.drambuffer.cpu_side = system.membus.master
    # system.drambuffer.mem_side = system.drambus.slave
    # system.mem_ctrls[0].port = system.drambus.master
    system.mem_ctrls[0].port = system.membus.master


root = Root(full_system = False, system = system)
Simulation.run(options, root, system, FutureClass)
