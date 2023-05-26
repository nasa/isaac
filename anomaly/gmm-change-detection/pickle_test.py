import pickle

word_list = ["cat", "hat", "jump", "house", "orange", "brick", "horse", "word"]

# do your thing here, like
word_list.append("monty")

# open a pickle file
filename = "mypickle.pk"

with open(filename, "wb") as fi:
    # dump your data into the file
    pickle.dump(word_list, fi)
