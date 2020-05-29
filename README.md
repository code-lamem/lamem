# LaMem: A Labeled Memory System

​	LaMem is implemented with Gem5 and NVMain simulators.  Gem5 is a modular platform for computer-system architecture research, encompassing system-level architecture as well as processor microarchitecture. NVMain is a cycle-accurate memory simulator, it models components of DRAM and NVMs, and memory hierarchy in detail. 

### System Simultor Gem5

To build gem5, you will need the following software: 

g++ or clang, Python (gem5 links in the Python interpreter), SCons, SWIG, zlib, m4, and lastly protobuf if you want trace capture and playback support. Please see http://www.gem5.org/Dependencies for more details concerning the minimum versions of the aforementioned tools.

clone gem5 source from gem5.org

```bash
[          ~]$ hg clone http://repo.gem5.org/gem5
```

build gem5

```bash
[gem5-source]$ scons build/X86/gem5.opt build/X86/gem5.debug
```

### Memory Simulator NVMain

You can download the NVMain source from:[https://bitbucket.org/mrp5060/nvmain/src/default/](https://links.jianshu.com/go?to=https%3A%2F%2Fbitbucket.org%2Fmrp5060%2Fnvmain%2Fsrc%2Fdefault%2F)

Initialize queues in gem5:

```bash
[gem5-source]$ hg qinit
```

Import the NVMain patch:

```bash
[gem5-source]$ hg qimport -f ./nvmain/patches/gem5/nvmain2-gem5-10688+
```

Apply the patch:

```bash
[gem5-source]$ hg qpush
```

###  Compile Gem5-NVMain

```bash
[gem5-source]$ scons EXTRAS=nvmain ./build/X86/gem5.opt
```

Based on the "Gem5 + NVMain" hybrid simulator, LaMem has added the following functions:

- **Implement a hierarchical DRAM cache layer between LLC and NVM(main memory) :** DRAM cache is managed by hardware in tranditional DRAM/NVM hierarchical hybrid systems.
- **Labeled requests :** a new hardware-software abstraction, serving as the basic unit of expressing the priority of application programs, to convey higher-level program semantics from applications to the system hardware. 
- **Resources isolation :** leveraging the way-based partitioning technique for shared resources to
  achieve resource isolation, while employing applications clustering technique to address the limitations of way-partitioning and achieve more performance gains.
- **BDServ allocation :**  BDServ aims to allocate resources to high-priority applications, while providing direct access to relatively-low-cost main memory for low-priority applications when the remainremaining resources are insufficient.

### 
