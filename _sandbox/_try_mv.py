import uctypes # type: ignore
from array import array
from rp2 import DMA # type: ignore
import time

buffer_n_samples = 128
# return array('H', [0] * (self.buffer_n_samples * 2))
# allocate a large buffer, twice the size of the required ping-pong buffer
_large_buffer = array('H', [0] * (buffer_n_samples * 4))  # four times the size for safety
# get the base address of the large buffer using uctypes
base_address = uctypes.addressof(_large_buffer)
# calculate an offset to align the base address to self.buffer_n_samples * 2
offset = (-base_address) & (buffer_n_samples * 2 - 1)
start = (base_address + buffer_n_samples * 2 - 1) & ~(buffer_n_samples * 2 - 1) # advance by buffer_n_samples * 2 -1 and then zero out the lower bits

print(f'Base address: {base_address}, Offset: {offset}, Aligned address: {base_address + offset}')
# print(f'Base address: {base_address}, Aligned address: {start}')


whole_mv = memoryview(_large_buffer)
aligned_mv = whole_mv[offset:offset + buffer_n_samples * 2]

# fill the underlying buffer with some data. Use 16-bit unsigned integers
for i in range(buffer_n_samples * 4):
    _large_buffer[i] = (i+40000) & 0xFFFF  # ensure values are within 16-bit unsigned integer range

# print both the large buffer and the aligned memoryview
print("Large buffer:", _large_buffer)
print("Aligned memoryview:", [x for x in aligned_mv])

# create another buffer, the size of the ping-pong buffer, to receive data from the aligned memoryview
copy_target_buffer = array('H', [0] * (buffer_n_samples * 2))
# print to show it's empty
print("Copy target buffer (before):", copy_target_buffer)
# copy data from the aligned memoryview to the ping-pong buffer
copy_target_buffer[:] = aligned_mv[:]
# print the ping-pong buffer to verify the copy
print("copy target buffer (after mem copy):", copy_target_buffer)
# change the first element of the copy target buffer, and compare it with the aligned memoryview to show they are different
copy_target_buffer[0] = 12345
print(f'First element of copy target buffer: {copy_target_buffer[0]}, first element of aligned memoryview: {aligned_mv[0]}')

# create yet another buffer, to be copied to by a DMA channel
dma_target_buffer = array('H', [0] * (buffer_n_samples * 2))
# print to show it's empty
print("DMA target buffer (before):", dma_target_buffer)
dma = DMA()
ctrl_value = dma.pack_ctrl(size=1, inc_write=True)
print(f'count parameter: {len(aligned_mv)}')
dma.config(
    read=aligned_mv,
    write=dma_target_buffer,
    count=len(aligned_mv),
    ctrl=ctrl_value,
    trigger=True
)
time.sleep_us(1)
while dma.count > 0:
    pass  # wait for DMA to finish
print("DMA target buffer (after DMA copy):", dma_target_buffer)
