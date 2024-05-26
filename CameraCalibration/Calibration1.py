import pickle

# Open the file in binary read mode
with open('calibration.pkl', 'rb') as file:
    # Use pickle to load the object from the file
    data = pickle.load(file)

# Now you can use the 'data' object as you have loaded it
print(data)
