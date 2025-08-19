import h5py

with h5py.File("/home/user/IsaacLab/datasets/datasets_test3.hdf5", "r") as f:
    def show(name, obj):
        if isinstance(obj, h5py.Dataset):
            print(f"{name}: shape={obj.shape}, dtype={obj.dtype}")
    f.visititems(show)
