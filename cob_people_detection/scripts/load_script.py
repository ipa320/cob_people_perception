import wx
import os
import sys


class dlg(wx.Frame):
  def __init__(self):
    self.base_path="/share/goa-tz/people_detection/eval/"
    self.ts_dir_list = list()
    self.pf_list = list()

    self.f=wx.Frame(None,title="Evaluation GUI",size=(200,500))


    self.makeLayout(self.f)
    self.makeBindings()
    self.f.Show()
  def makeBindings(self):
    self.dir_btn.Bind(wx.EVT_BUTTON,self.OnAddDir)
    self.pf_btn.Bind(wx.EVT_BUTTON,self.OnAddPf)
    self.ok_btn.Bind(wx.EVT_BUTTON,self.OnProcess)
    self.reset_btn.Bind(wx.EVT_BUTTON,self.OnReset)
    self.vis_btn.Bind(wx.EVT_BUTTON,self.OnRunVis)
  def makeLayout(self,parent):
    sizer=wx.GridSizer(8,1,0,0)

    pf_btn_txt=wx.StaticText(parent,-1,"Select probe file")
    self.pf_btn=wx.Button(parent,-1,"Browse",(70,30))

    dir_btn_txt=wx.StaticText(parent,-1,"Add directory to training set")
    self.dir_btn=wx.Button(parent,-1,"Browse",(70,30))

    reset_btn_txt=wx.StaticText(parent,-1,"Reset selections")
    self.reset_btn=wx.Button(parent,-1,"Reset",(70,30))

    ok_btn_txt=wx.StaticText(parent,-1,"Process training set")
    self.ok_btn=wx.Button(parent,-1,"Process",(70,30))

    vis_btn_txt=wx.StaticText(parent,-1,"Visualize Results")
    self.vis_btn=wx.Button(parent,-1,"Ok",(70,30))


    sizer.Add(dir_btn_txt,1,wx.BOTTOM |wx.ALIGN_BOTTOM)
    sizer.Add(self.dir_btn,1)
    sizer.Add(pf_btn_txt,1,wx.BOTTOM |wx.ALIGN_BOTTOM)
    sizer.Add(self.pf_btn,1)
    sizer.Add(reset_btn_txt,1,wx.BOTTOM | wx.ALIGN_BOTTOM)
    sizer.Add(self.reset_btn,1)
    sizer.Add(ok_btn_txt,1,wx.BOTTOM | wx.ALIGN_BOTTOM)
    sizer.Add(self.ok_btn,1)
    sizer.Add(vis_btn_txt,1,wx.BOTTOM | wx.ALIGN_BOTTOM)
    sizer.Add(self.vis_btn,1)

    parent.SetSizer(sizer)
#################### CALLBACK ###########################
  def OnRunVis(self,e):
    os.system("octave  pvis.m")
  def OnAddPf(self,e):
    self.pf_dlg=wx.FileDialog(None,"Select Probefile",defaultDir=self.base_path,style=wx.FD_MULTIPLE)
    if self.pf_dlg.ShowModal() == wx.ID_OK:
      temp_list= self.pf_dlg.GetPaths()
      for temp in temp_list:
        self.pf_list.append(temp)
  def OnAddDir(self,e):
    self.ts_dlg=wx.DirDialog(None,"Select directories")
    self.ts_dlg.SetPath(self.base_path)
    if self.ts_dlg.ShowModal() == wx.ID_OK:
      self.ts_dir_list.append(self.ts_dlg.GetPath())

  def OnProcess(self,e):
    self.process()

  def OnReset(self,e):
      self.reset()

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

    print "[EVAL TOOL] done"
if __name__=="__main__":
  app= wx.App(False)
  dlg = dlg()
  app.MainLoop()
