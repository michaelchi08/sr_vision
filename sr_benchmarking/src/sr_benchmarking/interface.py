#!/usr/bin/env python
# -*- coding: utf-8 -*-

import Tkinter as tk


class MainApplication(tk.Frame):
    """
    Graphical user interface for algorithm(s) and dataset(s) choosing
    """

    def __init__(self, parent):
        """
        Initialize the window with choice boxes and launch button
        @param parent - Tkinter frame object
        """
        tk.Frame.__init__(self, parent)
        self.parent = parent

        # frame 1
        self.Frame1 = tk.Frame(self.parent, borderwidth=2, relief=tk.GROOVE)
        algo_label = tk.Label(self.Frame1, text="Algorithm", relief=tk.GROOVE, font='Verdana 20 bold')
        algo_label.pack()
        self.algo_var = [tk.IntVar(0), tk.IntVar(0)]
        self.algo1 = tk.Checkbutton(self.Frame1, text="Finding blobs", variable=self.algo_var[0])
        self.algo2 = tk.Checkbutton(self.Frame1, text="Main colors", variable=self.algo_var[1])
        self.algo1.pack(padx=10, pady=5)
        self.algo2.pack(padx=10, pady=5)
        self.Frame1.grid(column=0, row=0, padx=10, pady=10)

        # frame 2
        self.Frame2 = tk.Frame(self.parent, borderwidth=2, relief=tk.GROOVE)
        data_label = tk.Label(self.Frame2, text="Dataset", relief=tk.GROOVE, font='Verdana 20 bold')
        data_label.pack()
        self.data_var = [tk.IntVar(0), tk.IntVar(0), tk.IntVar(0)]
        self.data1 = tk.Checkbutton(self.Frame2, text="Basic", variable=self.data_var[0])
        self.data2 = tk.Checkbutton(self.Frame2, text="Noise", variable=self.data_var[1])
        self.data3 = tk.Checkbutton(self.Frame2, text="Berkeley", variable=self.data_var[2])
        self.data1.pack(padx=10, pady=5)
        self.data2.pack(padx=10, pady=5)
        self.data3.pack(padx=10, pady=5)
        self.Frame2.grid(column=2, row=0, padx=10, pady=10)

        # frame 3
        self.Frame3 = tk.Frame(self.parent, borderwidth=2, relief=tk.GROOVE)
        self.bouton = tk.Button(self.Frame3, text="Lancer", command=self.launch, relief=tk.GROOVE, font='Verdana 20 ')
        self.bouton.pack()
        self.Frame3.grid(column=1, row=3, padx=10, pady=10)

    def launch(self):
        """
        Function called by the "Launch button"
        Save the choices and quit the interface
        """

        global algo_choice
        global data_choice
        algo_choice = [self.algo_var[0].get(), self.algo_var[1].get()]
        data_choice = [self.data_var[0].get(), self.data_var[1].get(), self.data_var[2].get()]
        if 1 not in algo_choice or 1 not in data_choice:
            pass
        else:
            self.parent.destroy()


class Result(tk.Frame):
    """
    Graphical interface for the results showing
    """

    def __init__(self, parent, res):
        """
        Initialize the window with the text box
        @param parent - Tkinter frame object
        @param res - Results as string
        """
        tk.Frame.__init__(self, parent)
        self.parent = parent
        self.box_text = tk.Text(self.parent)
        self.box_text.insert(tk.INSERT, res)
        self.box_text.pack(padx=10, pady=10)

class Result2:
    def __init__(self, res):
        self.root=Tk()
        self.root.title('Tableau - v.01')

        for k in range(4):
            basic=Button(self.root, text='Basic '+str(k+1))
            basic.grid(row=0,column=k+1,sticky=NSEW)
            noise=Button(self.root, text='Noise '+str(k+1))
            noise.grid(row=0,column=k+5,sticky=NSEW)
            berkeley=Button(self.root, text='Berkeley '+str(k+1))
            berkeley.grid(row=0,column=k+9,sticky=NSEW)
        self.root.grid_rowconfigure(0, weight=1)
        for m in range(2):
            b=Button(self.root, text=str(m+1), width=5)
            b.grid(row=m+1,column=0,sticky=NSEW)
        #self.root.grid_rowconfigure(0, weight=1)
        self.root.grid_columnconfigure(0, weight=1)

        for i in range(2):
            for j in range(12):
                e = Entry(self.root)
                e.grid(row=i+1, column=j+1)
                e.insert(INSERT, '(%d , %d)' % (i, j))
                self.root.grid_columnconfigure(j, weight=1)

            self.root.grid_rowconfigure(i+1, weight=1)
        self.root.mainloop()

def run_interface():
    """
    Run the main interface to chose and launch the benchmarking
    @return - User choices as tuples : 0=NO and 1=YES
    """
    root = tk.Tk()
    MainApplication(root)
    root.mainloop()
    return algo_choice, data_choice


def show_results(res):
    """
    Run the results showing window
    @param res - results as a string
    """
    root = tk.Tk()
    Result2(root, res)
    root.mainloop()
