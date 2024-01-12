import numpy as np

with open('x.npy', 'wb') as f:
    np.save(f, x)

with open('u.npy', 'wb') as f:
    np.save(f, u)
with open('t.npy', 'wb') as f:
    np.save(f, t)