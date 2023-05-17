import pickle

#filename = 'mypickle.pk'
filename = './saved_models/t_1.pk'

with open(filename, 'rb') as fi:
    word_list = pickle.load(fi)

print(word_list)
