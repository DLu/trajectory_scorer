import numpy
import matplotlib
import matplotlib.pyplot as plt
import pylab
from numpy import arange


def limit(labels, n):
    labels = ['%.1f'%x for x in labels]
    NL = [''] * len(labels)
    NL[0] = labels[0]
    NL[-1] = labels[-1]
    N = len(labels)
    for i in range(0, N, max(N/n,1)):
        NL[i] = labels[i]
    return NL

def heatmap(D, include_black=False, x_axis=None, y_axis=None, x_label='', y_label='', label=''):
    colors = [(pylab.cm.jet(i)) for i in xrange(1,255)] + [('white')] 
    colors.reverse()
    if include_black:
        colors.append('black')

    heatmap_custom_color(D, colors, x_axis, y_axis, x_label, y_label, label)

    
def heatmap_custom_color(D, color_list, x_axis=None, y_axis=None, x_label='', y_label='', label='', ax=None):
    mdata = numpy.array(D) 

    if ax is None:
        fig, ax = plt.subplots()
    
    color_map = matplotlib.colors.LinearSegmentedColormap.from_list('new_map', color_list, N=len(color_list))

    heatmap = ax.pcolor(mdata,cmap=color_map)

    if x_axis is not None:
        ax.set_xticks(arange(len(x_axis))+0.5, minor=False)
        ax.set_xticklabels(limit(x_axis,10))        
    if y_axis is not None:
        ax.set_yticks(arange(len(y_axis))+0.5, minor=False)
        ax.set_yticklabels(limit(y_axis,10))


    ax.set_title(label)
    plt.xlabel(x_label)
    plt.ylabel(y_label)

def show():
    plt.show()
    
