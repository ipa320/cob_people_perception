import wx
import os
import sys


class dlg(wx.Frame):
  def __init__(self):
    self.ts_dir_list = list()

    self.f=wx.Frame(None,title="Evaluation GUI",size=(500,500))


    self.makeLayout(self.f)
    self.makeBindings()
    self.f.Show()
  def makeBindings(self):
    self.dir_btn.Bind(wx.EVT_BUTTON,self.OnAddDir)
    self.ok_btn.Bind(wx.EVT_BUTTON,self.OnProcess)
  def makeLayout(self,parent):
    sizer=wx.GridSizer(4,1,0,0)

    dir_btn_txt=wx.StaticText(parent,-1,"Add directory to training set")
    self.dir_btn=wx.Button(parent,-1,"Add",(70,30))

    ok_btn_txt=wx.StaticText(parent,-1,"Process training set")
    self.ok_btn=wx.Button(parent,-1,"Process",(70,30))


    sizer.Add(dir_btn_txt,1,wx.BOTTOM |wx.ALIGN_BOTTOM)
    sizer.Add(self.dir_btn,1)
    sizer.Add(ok_btn_txt,1,wx.BOTTOM | wx.ALIGN_BOTTOM)
    sizer.Add(self.ok_btn,1)

    parent.SetSizer(sizer)
#################### CALLBACK ###########################
  def OnAddDir(self,e):
    self.ts_dlg=wx.DirDialog(None,"Choose files")
    self.ts_dlg.SetPath("/share/goa-tz/people_detection/eval/")
    if self.ts_dlg.ShowModal() == wx.ID_OK:
      self.ts_dir_list.append(self.ts_dlg.GetPath())

  def OnProcess(self,e):
    self.process()

  def process(self):

    base_path="/share/goa-tz/people_detection/eval/"
    training_set_list_path=base_path+"training_set_list"
    probe_file_list_path=base_path+"probe_file_list"
    class_overview_path=base_path+"class_overview"

    os.system("rm %s"%training_set_list_path)
    os.system("rm %s"%probe_file_list_path)
    os.system("rm %s"%class_overview_path)

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
          if file.endswith(".pgm"):
            if file.endswith("Ambient.pgm"):
              aaa=1
            else:
              # construct filepath
              file_path=db_path+dir+"/"+file
              training_set_file_stream.write(file_path)
              training_set_file_stream.write("\n")
        training_set_file_stream.write("$$\n")
        os.chdir(db_path)
if __name__=="__main__":
  app= wx.App(False)
  dlg = dlg()
  app.MainLoop()
