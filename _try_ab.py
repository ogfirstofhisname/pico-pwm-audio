import uctypes
from array import array

class AlignedBuffer:
    def __init__(self, buffer_n_samples, typecode='H', alignment=None):
        self.typecode = typecode
        self.element_size = 2 if typecode == 'H' else 1  # Adjust based on type
        self.buffer_n_samples = buffer_n_samples
        
        # Default alignment to buffer size in bytes
        if alignment is None:
            alignment = buffer_n_samples * 2 * self.element_size
        
        # Allocate oversized buffer for alignment
        oversized_length = buffer_n_samples * 4
        self._large_buffer = array(typecode, [0] * oversized_length)
        
        # Calculate alignment
        base_address = uctypes.addressof(self._large_buffer)
        offset_bytes = (-base_address) & (alignment - 1)
        offset_elements = offset_bytes // self.element_size
        
        # Create aligned memoryview
        whole_mv = memoryview(self._large_buffer)
        self._buffer = whole_mv[offset_elements:offset_elements + buffer_n_samples * 2]
        
        # Store addresses for verification
        self.base_address = base_address
        self.aligned_address = base_address + offset_bytes
        
    def __len__(self):
        return len(self._buffer)
    
    def __getitem__(self, key):
        return self._buffer[key]
    
    def __setitem__(self, key, value):
        self._buffer[key] = value
    
    def __iter__(self):
        return iter(self._buffer)
    
    def __repr__(self):
        return f"AlignedBuffer(length={len(self)}, address=0x{self.aligned_address:08x})"
    
    # Array-like methods
    @property
    def memoryview(self):
        """Access to underlying memoryview for DMA operations"""
        return self._buffer
    
    def fill(self, value):
        """Fill buffer with a value"""
        for i in range(len(self._buffer)):
            self._buffer[i] = value
    
    def copy_from(self, source):
        """Copy data from another buffer/sequence"""
        for i, val in enumerate(source):
            if i >= len(self._buffer):
                break
            self._buffer[i] = val
    
    def to_list(self):
        """Convert to list"""
        return list(self._buffer)
    
    def address(self):
        """Get the memory address of the aligned buffer"""
        return self.aligned_address