#!/usr/bin/env python
import wx
import os
import sys
import random

import math
from threading import Thread


class dlg(wx.Frame):
  def __init__(self):

    # varables for gui
    #self.bin_path="/home/tom/git/care-o-bot/cob_people_perception/cob_people_detection/bin/"
    #self.base_path="/home/tom/git/care-o-bot/cob_people_perception/cob_people_detection/debug/eval/"
    self.base_path="/share/goa-tz/people_detection/eval/"
    self.bin_path="/home/goa-tz/git/care-o-bot/cob_people_perception/cob_people_detection/bin/"

    self.Evaluator=Evaluator()


    self.output_path=os.path.join(self.base_path,"eval_tool_files")
    self.cwd=os.getcwd()
    self.ts_dir_list = list()
    self.f=wx.Frame(None,title="Evaluation GUI",size=(650,500))

    # variables for output
    self.pf_list = list()
    self.ts_list = list()
    self.cl_list = list()



    self.makeLayout(self.f)
    self.makeBindings()
    self.f.Show()
  def makeBindings(self):
    self.dir_btn.Bind(wx.EVT_BUTTON,self.OnAddDir)
    self.pf_btn.Bind(wx.EVT_BUTTON,self.OnAddPf)
    self.ok_btn.Bind(wx.EVT_BUTTON,self.OnProcess)
    self.reset_btn.Bind(wx.EVT_BUTTON,self.OnReset)
    self.vis_btn.Bind(wx.EVT_BUTTON,self.OnRunVis)
    self.eval_btn.Bind(wx.EVT_BUTTON,self.OnEvaluate)


    self.del_dir_btn.Bind(wx.EVT_BUTTON,lambda evt,gl=self.ts_glist,l=self.ts_dir_list :self.OnResetList(evt,l,gl))
    self.del_pf_btn .Bind(wx.EVT_BUTTON,lambda evt,gl=self.pf_glist,l=self.pf_list :self.OnResetList(evt,l,gl))

  def makeLayout(self,parent):
    pf_btn_txt=wx.StaticText(parent,-1,"Select probe file")
    self.pf_btn=wx.Button(parent,-1,"Browse",(70,30))

    del_pf_btn_txt=wx.StaticText(parent,-1,"Delete probe file")
    self.del_pf_btn=wx.Button(parent,-1,"Reset",(70,30))

    dir_btn_txt=wx.StaticText(parent,-1,"Add training set")
    self.dir_btn=wx.Button(parent,-1,"Browse",(70,30))

    del_dir_btn_txt=wx.StaticText(parent,-1,"Delete Database")
    self.del_dir_btn=wx.Button(parent,-1,"Reset",(70,30))

    reset_btn_txt=wx.StaticText(parent,-1,"Delete All")
    self.reset_btn=wx.Button(parent,-1,"Reset",(70,30))

    ok_btn_txt=wx.StaticText(parent,-1,"Process Config")
    self.ok_btn=wx.Button(parent,-1,"Process",(70,30))

    vis_btn_txt=wx.StaticText(parent,-1,"Visualize")
    self.vis_btn=wx.Button(parent,-1,"Ok",(70,30))

    eval_btn_txt=wx.StaticText(parent,-1,"Evaluate")
    self.eval_btn=wx.Button(parent,-1,"Ok",(70,30))


    protocol_choice_txt=wx.StaticText(parent,-1,"Select Protocol")
    self.protocol_choice=wx.Choice(parent,-1,choices=["leave one out","leave half out","manual selection"])


    #spin_rep_txt=wx.StaticText(parent,-1,"Repetitions")
    self.spin_rep=wx.SpinCtrl(parent,-1,size=wx.Size(50,30),min=1,max=60)

    #classifier_choice_txt=wx.StaticText(parent,-1,"Select Classifier")
    self.classifier_choice=wx.Choice(parent,-1,choices=["KNN","SVM","MIN DIFFS"])

    method_choice_txt=wx.StaticText(parent,-1,"Select Method")
    self.method_choice=wx.Choice(parent,-1,choices=["Fisherfaces","Eigenfaces"])

    self.nrm_checkbox=wx.CheckBox(parent,label="normalize")

    # visual feedback lists
    self.ts_glist=wx.ListBox(choices=[],id=-1,parent=parent,size=wx.Size(80,100))
    self.pf_glist=wx.ListBox(choices=[],id=-1,parent=parent,size=wx.Size(80,100))

    # dummy for filling empty spaces
    dummy=wx.StaticText(parent,-1,'')

    # Change to flexgridsizer
    ##sizer=wx.GridSizer(8,3,0,0)
    sizer=wx.FlexGridSizer(10,3,0,0)
    sizer.AddGrowableCol(2)

    sizer.Add(dir_btn_txt,1,wx.BOTTOM |wx.ALIGN_BOTTOM)
    sizer.Add(del_dir_btn_txt,1,wx.BOTTOM |wx.ALIGN_BOTTOM)
    sizer.Add(dummy,1,wx.EXPAND)

    sizer.Add(self.dir_btn,1)
    sizer.Add(self.del_dir_btn,1)
    sizer.Add(self.ts_glist,1,wx.EXPAND)


    sizer.Add(protocol_choice_txt,1,wx.BOTTOM |wx.ALIGN_BOTTOM)
    sizer.Add(self.protocol_choice,1)
    sizer.Add(self.spin_rep,1)



    sizer.Add(pf_btn_txt,1,wx.BOTTOM |wx.ALIGN_BOTTOM)
    sizer.Add(del_pf_btn_txt,1,wx.BOTTOM |wx.ALIGN_BOTTOM)
    sizer.Add(dummy,1,wx.EXPAND)

    sizer.Add(self.pf_btn,1)
    sizer.Add(self.del_pf_btn,1)
    sizer.Add(self.pf_glist,1,wx.EXPAND)


    sizer.Add(ok_btn_txt,1,wx.BOTTOM | wx.ALIGN_BOTTOM)
    sizer.Add(dummy,1,wx.EXPAND)
    sizer.Add(method_choice_txt,1,wx.BOTTOM | wx.ALIGN_BOTTOM)
    #sizer.Add(classifier_choice_txt,1,wx.BOTTOM | wx.ALIGN_BOTTOM)

    sizer.Add(self.ok_btn,1)
    sizer.Add(dummy,1,wx.EXPAND)

    bs=wx.BoxSizer(wx.HORIZONTAL)
    bs.Add(self.method_choice,1)
    bs.Add(self.classifier_choice,1)
    bs.Add(self.nrm_checkbox,1)
    sizer.Add(bs,1,wx.BOTTOM | wx.ALIGN_BOTTOM)

    sizer.Add(vis_btn_txt,1,wx.BOTTOM | wx.ALIGN_BOTTOM)
    sizer.Add(eval_btn_txt,1,wx.BOTTOM | wx.ALIGN_BOTTOM)
    sizer.Add(reset_btn_txt,1,wx.BOTTOM | wx.ALIGN_BOTTOM)

    sizer.Add(self.vis_btn,1)
    sizer.Add(self.eval_btn,1)
    sizer.Add(self.reset_btn,1)

    parent.SetSizer(sizer)
#################### CALLBACK ###########################
  def OnEvaluate(self,e):
    self.evaluate()

  def OnRunVis(self,e):
    script_path=self.cwd+"/pvis.m"
    os.system("octave  %s"%script_path)
    os.chdir(self.base_path+"/vis")
    os.system("eog clustering_FF.jpg ")
    os.chdir(self.cwd)
  def OnAddPf(self,e):
    self.pf_dlg=wx.FileDialog(None,"Select Probefile",defaultDir=self.base_path,style=wx.FD_MULTIPLE)
    if self.pf_dlg.ShowModal() == wx.ID_OK:
      temp_list= self.pf_dlg.GetPaths()
      for temp in temp_list:
        self.pf_glist.Append(temp)

  def OnAddDir(self,e):
    self.ts_dlg=wx.DirDialog(None,"Select directories")
    self.ts_dlg.SetPath(self.base_path)
    if self.ts_dlg.ShowModal() == wx.ID_OK:
      self.ts_dir_list.append(self.ts_dlg.GetPath())
      self.ts_glist.Append(self.ts_dir_list[-1])

  def OnProcess(self,e):
    # get method and classifer
    method=str()
    classifier=str()
    if self.method_choice.GetCurrentSelection()==0:
      method="FISHER"
    elif self.method_choice.GetCurrentSelection()==1:
      method="EIGEN"

    if self.classifier_choice.GetCurrentSelection()==0:
      classifier="KNN"
    elif self.classifier_choice.GetCurrentSelection()==1:
      classifier="SVM"
    elif self.classifier_choice.GetCurrentSelection()==2:
      classifier="DIFFS"


    if len(self.ts_dir_list)>0:
      if(self.protocol_choice.GetCurrentSelection()==0):
        self.process_protocol(self.process_leave_1_out,method,classifier)
        output_file=os.path.join(self.output_path,"eval_file")
        for i in xrange(self.spin_rep.GetValue()-1):
          self.process_protocol(self.process_leave_1_out,method,classifier)
          os.rename(output_file,output_file+str(i))

      elif(self.protocol_choice.GetCurrentSelection()==1):
        self.process_protocol(self.process_leave_half_out,method,classifier)
        output_file=os.path.join(self.output_path,"eval_file")
        for i in xrange(self.spin_rep.GetValue()-1):
          self.process_protocol(self.process_leave_half_out,method,classifier)
          os.rename(output_file,output_file+str(i))

      elif(self.protocol_choice.GetCurrentSelection()==2):
        self.process_protocol(self.process_manual,method,classifier)

    print self.Evaluator.calc_stats()
    self.Evaluator.reset()

  def OnReset(self,e):
      self.delete_files()


  def OnResetList(self,e,l,gl):
    del l[:]
    gl.Clear()

  def delete_files(self):
    os.chdir(self.base_path)
    reset_str="rm "+os.path.join(self.output_path,"*")
    os.system(reset_str)

  def reset_lists(self):
    del self.ts_list[:]
    del self.pf_list[:]
    del self.cl_list[:]



#*****************************************************
#****************Internal Functions********************
#*****************************************************

  def process_protocol(self,protocol_fn,method,classifier):
    self.reset_lists()
    self.file_ops(self.make_ts_list)
    self.file_ops(self.make_cl_list)
    protocol_fn()
    self.sync_lists()
    self.print_lists()
    os.chdir(self.bin_path)


    if self.nrm_checkbox.GetValue()==True:
      bin_str="./ssa_test "+method+" "+classifier+" 1"
    else:
      bin_str="./ssa_test "+method+" "+classifier
    t=Thread(target=self.run_bin,args=(bin_str,))
    t.start()
    t.join()

    os.chdir(self.cwd)
    self.evaluate()


  def run_bin(self,i_str):
    os.system(i_str)


  def process_leave_half_out(self):
    self.file_ops(self.leave_k_out,"half")

  def process_leave_1_out(self):
    self.file_ops(self.leave_k_out,1)

  def process_manual(self):
    self.pf_list=[[] for i in range(len(self.cl_list))]
    self.pf_list_format(self.pf_glist.GetItems())

  def file_ops(self,fn=-1,add_param=False):

    if fn ==-1:
      def fn(x):
        return x

    for db in self.ts_dir_list:
      db_path=db
      os.chdir(db_path)

      dir_list=os.listdir(".")
      # enter directory - class
      for dir in dir_list:
        file_list_valid=list()
        os.chdir(dir)
        file_list_all=os.listdir(".")

        # loop through all files
        for file in file_list_all:
          if file.endswith(".bmp") or file.endswith(".jpg") or file.endswith(".pgm") or file.endswith(".png"):
            if file.endswith("Ambient.pgm"):
              aaa=1
            else:
              # construct filepath
              file_path=db_path+"/"+dir+"/"+file
              file_list_valid.append(file_path)
        if add_param==False:
          fn(file_list_valid)
        else:
          fn(file_list_valid,add_param)
        os.chdir(db_path)


  def pf_list_format(self,file_list):
    for cl in xrange(len(self.ts_list)):
      for i in xrange(len(file_list)):
        if file_list[i] in self.ts_list[cl]:
          self.pf_list[cl].append(file_list[i])

  def leave_k_out(self,file_list_valid,k):
        num_samples=len(file_list_valid)
        if(k is "half"):
          k=num_samples/2
        success_ctr=0
        rnd_list=list()
        pf_list=list()
        while len(rnd_list)<k:
          rnd_no=random.randint(0,num_samples-1)
          if not rnd_no in rnd_list :
            pf_list.append(file_list_valid[rnd_no])
            rnd_list.append(rnd_no)
        self.pf_list.append(pf_list)

  def make_ts_list(self,file_list):
      self.ts_list.append(file_list)

  def make_cl_list(self,file_list):
    class_name=os.path.basename(os.getcwd())
    self.cl_list.append(class_name)


  def sync_lists(self):
    for c in xrange(len(self.ts_list)):
      for s in self.pf_list[c]:
        self.ts_list[c].remove(s)


  def evaluate(self):

    results=list()
    groundtruth=list()
    files=list()

    input_path=os.path.join(self.output_path,"classified_output")
    eval_file_path=os.path.join(self.output_path,"eval_file")
    with open(input_path,"r") as input_stream:
      classified_list=input_stream.read().splitlines()

    with open(eval_file_path,"w") as eval_file_stream:
      cont_ctr=0
      for pf_l in xrange(len(self.pf_list)):
        for pf in self.pf_list[pf_l]:
          o_str = pf+" "+str(pf_l)+" "+str(classified_list[cont_ctr])+"\n"
          eval_file_stream.write(o_str)

          results.append(classified_list[cont_ctr])
          groundtruth.append(pf_l)
          files.append(pf)

          cont_ctr+=1
    self.Evaluator.add_epoch(groundtruth,results,files)


  def print_lists(self):

    print "[EVAL TOOL] creating lists"
    training_set_list_path=os.path.join(self.output_path,"training_set_list")
    probe_file_list_path=os.path.join(self.output_path,"probe_file_list")
    class_overview_path=os.path.join(self.output_path,"class_overview")


    training_set_file_stream = open(training_set_list_path,"w")
    probe_file_stream = open(probe_file_list_path,"w")
    class_overview_stream = open(class_overview_path,"w")

    for c in xrange(len(self.ts_list)):
      for s in self.ts_list[c]:
        training_set_file_stream.write(s)
        training_set_file_stream.write("\n")
      training_set_file_stream.write("$$\n")

    for c in xrange(len(self.pf_list)):
      for s in self.pf_list[c]:
        probe_file_stream.write(s)
        probe_file_stream.write("\n")

    for c in xrange(len(self.cl_list)):
      o_str=str(c)+" - "+self.cl_list[c]+"\n"
      class_overview_stream.write(o_str) 

#---------------------------------------------------------------------------------------
#----------------------------EVALUATOR-----------------------------------------
#---------------------------------------------------------------------------------------

class epoch():
  def __init__(self,gt,res,desc,ctr):
    self.gt=gt
    self.res=res
    self.desc=desc
    self.error_rate=float()

    self.calc_error_rate()

  def calc_error_rate(self):
    error_list=list()
    for i in xrange(len(self.gt)):
        if (int(self.gt[i]) == int(self.res[i])):
          error_list.append(0)
        else:
          error_list.append(1)

    n=len(error_list)
    self.error_rate=float(sum(error_list))/float(n)

class Evaluator():
  def __init__(self):
    print "EVALUATOR instantiated"
    self.epochs=list()
    self.epoch_ctr=0

  def reset(self):
    self.epoch_ctr=0
    del self.epochs[:]


  def add_epoch(self,gt,res,files):
    e=epoch(gt,res,files,self.epoch_ctr)
    self.epoch_ctr+=1
    self.epochs.append(e)

  def calc_stats(self):
    n=float(len(self.epochs))
    err=0.0
    mean_error_rate=0.0
    for e in self.epochs:
      err+=e.error_rate
    mean_error_rate=err/n


    if len(self.epochs)>1:
      v2=0.0
      for e in self.epochs:
        v2+=(e.error_rate - mean_error_rate)*(e.error_rate - mean_error_rate)
      sigma=math.sqrt(1/(n-1)*v2)
    else:
      sigma=0.0


    stats={"succes_rate":1-mean_error_rate,"m_err":mean_error_rate,"sigma":sigma}
    return stats





if __name__=="__main__":
  app= wx.App(False)
  dlg = dlg()
  #E=Evaluator()
  #a=list([2,3,3,4])
  #b=list([1,2,3,4])
  #d=list([2,3,3,4])
  #c=list(["1","2","3","4"])
  #E.add_epoch(a,b,c)
  #E.add_epoch(a,d,c)
  #print E.mean_error_rate()
  app.MainLoop()
