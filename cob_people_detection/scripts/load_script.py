#!/usr/bin/env python
import wx
import os
import sys


class dlg(wx.Frame):
  def __init__(self):
    self.base_path="/share/goa-tz/people_detection/eval/"
    self.bin_path="/home/goa-tz/git/care-o-bot/cob_people_perception/cob_people_detection/bin/"
    self.cwd=os.getcwd()
    self.ts_dir_list = list()
    self.pf_list = list()

    self.f=wx.Frame(None,title="Evaluation GUI",size=(650,500))


    self.makeLayout(self.f)
    self.makeBindings()
    self.f.Show()
  def makeBindings(self):
    self.dir_btn.Bind(wx.EVT_BUTTON,self.OnAddDir)
    self.pf_btn.Bind(wx.EVT_BUTTON,self.OnAddPf)
    self.ok_btn.Bind(wx.EVT_BUTTON,self.OnProcess)
    self.reset_btn.Bind(wx.EVT_BUTTON,self.OnReset)
    self.vis_btn.Bind(wx.EVT_BUTTON,self.OnRunVis)


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

    reset_btn_txt=wx.StaticText(parent,-1,"Delete Config")
    self.reset_btn=wx.Button(parent,-1,"Reset",(70,30))

    ok_btn_txt=wx.StaticText(parent,-1,"Process Config")
    self.ok_btn=wx.Button(parent,-1,"Process",(70,30))

    vis_btn_txt=wx.StaticText(parent,-1,"Visualize Results")
    self.vis_btn=wx.Button(parent,-1,"Ok",(70,30))


    # visual feedback lists
    self.ts_glist=wx.ListBox(choices=[],id=-1,parent=parent,size=wx.Size(80,100))
    self.pf_glist=wx.ListBox(choices=[],id=-1,parent=parent,size=wx.Size(80,100))

    # dummy for filling empty spaces
    dummy=wx.StaticText(parent,-1,'')

    # Change to flexgridsizer
    ##sizer=wx.GridSizer(8,3,0,0)
    sizer=wx.FlexGridSizer(8,3,0,0)
    sizer.AddGrowableCol(2)

    sizer.Add(dir_btn_txt,1,wx.BOTTOM |wx.ALIGN_BOTTOM)
    sizer.Add(del_dir_btn_txt,1,wx.BOTTOM |wx.ALIGN_BOTTOM)
    sizer.Add(dummy,1,wx.EXPAND)

    sizer.Add(self.dir_btn,1)
    sizer.Add(self.del_dir_btn,1)
    sizer.Add(self.ts_glist,1,wx.EXPAND)


    sizer.Add(pf_btn_txt,1,wx.BOTTOM |wx.ALIGN_BOTTOM)
    sizer.Add(del_pf_btn_txt,1,wx.BOTTOM |wx.ALIGN_BOTTOM)
    sizer.Add(dummy,1,wx.EXPAND)

    sizer.Add(self.pf_btn,1)
    sizer.Add(self.del_pf_btn,1)
    sizer.Add(self.pf_glist,1,wx.EXPAND)


    sizer.Add(ok_btn_txt,1,wx.BOTTOM | wx.ALIGN_BOTTOM)
    sizer.Add(reset_btn_txt,1,wx.BOTTOM | wx.ALIGN_BOTTOM)
    sizer.Add(dummy,1,wx.EXPAND)

    sizer.Add(self.ok_btn,1)
    sizer.Add(self.reset_btn,1)
    sizer.Add(dummy,1,wx.EXPAND)

    sizer.Add(vis_btn_txt,1,wx.BOTTOM | wx.ALIGN_BOTTOM)
    sizer.Add(dummy,1,wx.EXPAND)
    sizer.Add(dummy,1,wx.EXPAND)

    sizer.Add(self.vis_btn,1)
    sizer.Add(dummy,1,wx.EXPAND)
    sizer.Add(dummy,1,wx.EXPAND)

    parent.SetSizer(sizer)
#################### CALLBACK ###########################
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
        self.pf_list.append(temp)
        self.pf_glist.Append(temp)

  def OnAddDir(self,e):
    self.ts_dlg=wx.DirDialog(None,"Select directories")
    self.ts_dlg.SetPath(self.base_path)
    if self.ts_dlg.ShowModal() == wx.ID_OK:
      self.ts_dir_list.append(self.ts_dlg.GetPath())
      self.ts_glist.Append(self.ts_dir_list[-1])

  def OnProcess(self,e):
    if len(self.ts_dir_list)>0:
      self.process()
    os.chdir(self.bin_path)
    os.system("./ssa_test")
    os.chdir(self.cwd)

  def OnReset(self,e):
      self.reset()


  def OnResetList(self,e,l,gl):
    del l[:]
    gl.Clear()

  def reset(self):
    os.chdir(self.base_path)
    os.system("rm *")
    self.pf_list=list()
    self.ts_dir_list=list()



  def process(self):

    print "[EVAL TOOL] creating lists"
    training_set_list_path=self.base_path+"training_set_list"
    probe_file_list_path=self.base_path+"probe_file_list"
    class_overview_path=self.base_path+"class_overview"


    training_set_file_stream = open(training_set_list_path,"w")
    probe_file_stream = open(probe_file_list_path,"w")
    class_overview_stream = open(class_overview_path,"w")
    #  zero based class index
    class_index = 0

    for db in self.ts_dir_list:
      db_path=db
      os.chdir(db_path)

     ## TODO: Asser that only directories are chosen 
      dir_list=os.listdir(".")

      # enter directory - class
      for dir in dir_list:
        os.chdir(dir)
        file_list_all=os.listdir(".")

        # write class index 
        class_overview_stream.write(dir+" - "+ str(class_index)+"\n")
        class_index+=1

        # loop through all files
        for file in file_list_all:
          if file.endswith(".bmp") or file.endswith(".jpg") or file.endswith(".pgm") or file.endswith(".png"):
            if file.endswith("Ambient.pgm"):
              aaa=1
            else:
              # construct filepath
              file_path=db_path+"/"+dir+"/"+file
              training_set_file_stream.write(file_path)
              training_set_file_stream.write("\n")
        training_set_file_stream.write("$$\n")
        os.chdir(db_path)

    # fill probe file list
    for pf in self.pf_list:
        probe_file_stream.write(pf)
        probe_file_stream.write("\n")

    os.chdir(self.cwd)
    print "[EVAL TOOL] done"
if __name__=="__main__":
  app= wx.App(False)
  dlg = dlg()
  app.MainLoop()
