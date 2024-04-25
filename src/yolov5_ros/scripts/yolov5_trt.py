import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit
import numpy as np

# 加载TensorRT引擎模型
engine_file_path = 'path_to_your_engine_file.engine'
runtime = trt.Runtime(trt.Logger(trt.Logger.INFO))
with open(engine_file_path, 'rb') as f, runtime.deserialize_cuda_engine(f.read()) as engine:
    # 创建执行上下文
    context = engine.create_execution_context()

    # 创建输入和输出缓冲区
    input_shape = (1, 3, 224, 224)  # 输入数据的形状
    input_size = np.prod(input_shape)
    input_host = cuda.pagelocked_empty(input_size, dtype=np.float32)
    input_device = cuda.mem_alloc(input_host.nbytes)
    output_shape = (1, 1000)  # 输出数据的形状
    output_size = np.prod(output_shape)
    output_host = cuda.pagelocked_empty(output_size, dtype=np.float32)
    output_device = cuda.mem_alloc(output_host.nbytes)

    # 准备输入数据
    input_data = np.random.rand(*input_shape).astype(np.float32)
    np.copyto(input_host, input_data.ravel())

    # 将输入数据从主机复制到设备
    cuda.memcpy_htod(input_device, input_host)

    # 执行推理
    context.execute(1, [int(input_device)], [int(output_device)])

    # 将输出数据从设备复制到主机
    cuda.memcpy_dtoh(output_host, output_device)

    # 处理输出数据
    output_data = output_host.reshape(output_shape)

    # 打印输出数据
    print(output_data)
s