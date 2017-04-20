import numpy as np
from pycuda import driver, compiler, gpuarray, tools

import pycuda.autoinit
#This creates the kernel in python
kernel_code_template = """
__global__ void MatrixMulKernel(float *a, float *b, float *c)
{
    // 2D Thread ID (assuming that only *one* block will be executed)
    int tx = threadIdx.x;
    int ty = threadIdx.y;

    // Pvalue is used to store the element of the matrix
    // that is computed by the thread
    float Pvalue = 0;

    // Each thread loads one row of M and one column of N, 
    //   to produce one element of P.
    for (int k = 0; k < %(MATRIX_SIZE)s; ++k) {
        float Aelement = a[ty * %(MATRIX_SIZE)s + k];
        float Belement = b[k * %(MATRIX_SIZE)s + tx];
        Pvalue += Aelement * Belement;
    }

    // Write the matrix to device memory;
    // each thread writes one element
    c[ty * %(MATRIX_SIZE)s + tx] = Pvalue;
}
"""


#Define the number of blocks, link:http://documen.tician.de/pycuda/util.html#pycuda.tools.DeviceData
MATRIX_SIZE = 2

# Create two random square matrices of size 2x2 on the CPU memory using numpy
#It is intersting to note here that we need to cast it to single point float as this is done for a 32bit system,
#(I installed the 32 bit version) For latest versions of cuda, it can be done for double point precision floats too
a = np.random.randn(MATRIX_SIZE, MATRIX_SIZE).astype(np.float32)
b = np.random.randn(MATRIX_SIZE, MATRIX_SIZE).astype(np.float32)

#For direct computation on CPU
c = np.dot(a, b)

# transfer host (CPU) memory to device (GPU) memory 
a_d = gpuarray.to_gpu(a) 
b_d = gpuarray.to_gpu(b)

# Result matrix computed on the GPU
c_d = gpuarray.empty((MATRIX_SIZE, MATRIX_SIZE), np.float32)

#Kernel code
kernel = kernel_code_template % {
    'MATRIX_SIZE': MATRIX_SIZE 
    }

#Run the kernel 
mod = compiler.SourceModule(kernel)

# get the kernel function from the compiled module
matrixmul = mod.get_function("MatrixMulKernel")

# call the kernel on the card
matrixmul(
    a_d, b_d, 
    c_d, 
    
    block = (MATRIX_SIZE, MATRIX_SIZE, 1),
    )

# print the results


print a_d.get()
print b_d.get()
print c_d.get()

print "CPUvsGPU diff:"
print c - c_gpu.get()

np.allclose(c, c_d.get())