#!/usr/bin/env python3

"""
  PyPose: Bioloid pose system for arbotiX robocontroller
  Copyright (c) 2008-2010 Michael E. Ferguson.  All right reserved.

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software Foundation,
  Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
"""

import wx
import sys
import serial
import wx.adv
from serial.tools import list_ports

from pypose import ax12
from pypose.tools import panels
from pypose.driver import Driver
from pypose.project import Project


VERSION = "PyPose v2 / NUKE 0015"


class Editor(wx.Frame):
    """Implements the main editor window. """
    ID_NEW = wx.NewIdRef()
    ID_OPEN = wx.NewIdRef()
    ID_SAVE = wx.NewIdRef()
    ID_SAVE_AS = wx.NewIdRef()
    ID_EXIT = wx.NewIdRef()
    ID_EXPORT = wx.NewIdRef()
    ID_RELAX = wx.NewIdRef()
    ID_PORT = wx.NewIdRef()
    ID_ABOUT = wx.NewIdRef()
    ID_TEST = wx.NewIdRef()
    ID_TIMER = wx.NewIdRef()
    ID_COL_MENU = wx.NewIdRef()
    ID_LIVE_UPDATE = wx.NewIdRef()
    ID_2COL = wx.NewIdRef()
    ID_3COL = wx.NewIdRef()
    ID_4COL = wx.NewIdRef()

    def __init__(self):
        """ Creates pose editor window. """
        wx.Frame.__init__(self, None, -1, VERSION,
                          style=wx.DEFAULT_FRAME_STYLE)

        # key data for our program
        self.project = Project()  # holds data for our project
        self.panelIndex = dict()  # existant tools
        self.saveReq = False
        self.panel = None
        self.port = None
        self.filename = ""
        self.dirname = ""
        self.columns = 2        # column count for pose editor

        # for clearing red color on status bar
        self.timer = wx.Timer(self, self.ID_TIMER)
        self.timeout = 0

        # build our menu bar
        menubar = wx.MenuBar()
        prjmenu = wx.Menu()
        # dialog with name, # of servos
        prjmenu.Append(self.ID_NEW, "&New\tCtrl+N", "", wx.ITEM_NORMAL)
        prjmenu.Append(self.ID_OPEN, "&Open\tCtrl+O", "",
                       wx.ITEM_NORMAL)  # open file dialog
        prjmenu.AppendSeparator()
        # if name unknown, ask, otherwise save
        prjmenu.Append(self.ID_SAVE, "&Save\tCtrl+S", "", wx.ITEM_NORMAL)
        prjmenu.Append(self.ID_SAVE_AS, "Save As")  # ask for name, save
        prjmenu.AppendSeparator()
        prjmenu.Append(self.ID_EXIT, "&Quit\tCtrl+Q", "", wx.ITEM_NORMAL)
        menubar.Append(prjmenu, "Project")

        # Create tool panel
        toolsmenu = wx.Menu()
        for panel in panels:
            name = panel.NAME
            id = wx.NewIdRef()
            self.panelIndex[id] = panel
            toolsmenu.Append(id, name)
        toolsmenu.Append(self.ID_EXPORT, "Export to AVR")  # save as dialog
        menubar.Append(toolsmenu, "Tools")

        configmenu = wx.Menu()
        # dialog box: arbotix/thru, speed, port
        configmenu.Append(self.ID_PORT, "Port")
        columnmenu = wx.Menu()
        columnmenu.Append(self.ID_2COL, "2 columns")
        columnmenu.Append(self.ID_3COL, "3 columns")
        columnmenu.Append(self.ID_4COL, "4 columns")
        configmenu.AppendSubMenu(columnmenu, "Pose editor")
        # live update
        self.live = configmenu.Append(
            self.ID_LIVE_UPDATE, "Live pose update", kind=wx.ITEM_CHECK)
        # configmenu.Append(self.ID_TEST,"test") # for in-house testing of boards
        menubar.Append(configmenu, "Configuration")

        helpmenu = wx.Menu()
        helpmenu.Append(self.ID_ABOUT, "About")
        menubar.Append(helpmenu, "Help")

        self.SetMenuBar(menubar)

        # configure events
        self.Bind(wx.EVT_MENU, self.newFile, self.ID_NEW)
        self.Bind(wx.EVT_MENU, self.openFile, self.ID_OPEN)
        self.Bind(wx.EVT_MENU, self.saveFile, self.ID_SAVE)
        self.Bind(wx.EVT_MENU, self.saveFileAs, self.ID_SAVE_AS)
        self.Bind(wx.EVT_MENU, sys.exit, self.ID_EXIT)

        for t in self.panelIndex.keys():
            self.Bind(wx.EVT_MENU, self.loadTool, t)
        self.Bind(wx.EVT_MENU, self.export, self.ID_EXPORT)

        self.Bind(wx.EVT_MENU, self.doRelax, self.ID_RELAX)
        self.Bind(wx.EVT_MENU, self.doPort, self.ID_PORT)
        self.Bind(wx.EVT_MENU, self.doTest, self.ID_TEST)
        self.Bind(wx.EVT_MENU, self.doAbout, self.ID_ABOUT)
        self.Bind(wx.EVT_CLOSE, self.doClose)
        self.Bind(wx.EVT_TIMER, self.OnTimer, id=self.ID_TIMER)

        self.Bind(wx.EVT_MENU, self.setLiveUpdate, self.ID_LIVE_UPDATE)
        self.Bind(wx.EVT_MENU, self.do2Col, self.ID_2COL)
        self.Bind(wx.EVT_MENU, self.do3Col, self.ID_3COL)
        self.Bind(wx.EVT_MENU, self.do4Col, self.ID_4COL)

        # editor area
        self.sb = self.CreateStatusBar(2)
        self.sb.SetStatusWidths([-1, 250])
        self.sb.SetStatusText('not connected', 1)

        self.loadTool()
        self.sb.SetStatusText('please create or open a project...', 0)
        self.Centre()

        self.Show(True)

    def loadTool(self, e=None):
        """Load toolpane."""
        if self.panel is not None:
            self.panel.save()
            # self.sizer.Remove(self.panel)
            self.panel.Destroy()
        self.ClearBackground()
        # instantiate
        if e is None:
            # PoseEditor
            self.panel = panels[0](self, self.port)
        else:
            self.panel = self.panelIndex[e.GetId()](self, self.port)
        self.sizer = wx.BoxSizer(wx.VERTICAL)
        self.sizer.Add(self.panel, 1, wx.EXPAND | wx.ALL, 10)
        self.SetSizer(self.sizer)
        self.SetAutoLayout(1)
        self.sizer.Fit(self)
        self.sb.SetStatusText(self.panel.STATUS)
        self.panel.SetFocus()

    def newFile(self, e):
        """Open a dialog that asks for robot name and servo count."""
        dlg = NewProjectDialog(self, -1, "Create New Project")
        if dlg.ShowModal() == wx.ID_OK:
            self.project.new(dlg.name.GetValue(), dlg.count.GetValue(), int(
                dlg.resolution.GetValue()))
            self.loadTool()
            self.sb.SetStatusText('created new project ' + self.project.name + ', please create a pose...')
            self.SetTitle(VERSION + " - " + self.project.name)
            self.panel.saveReq = True
            self.filename = ""
        dlg.Destroy()

    def openFile(self, e):
        """Loads a robot file into the GUI."""
        dlg = wx.FileDialog(self, "Choose a file",
                            self.dirname, "", "*.ppr", wx.FD_OPEN)
        if dlg.ShowModal() == wx.ID_OK:
            self.filename = dlg.GetPath()
            self.dirname = dlg.GetDirectory()
            print("Opening: " + self.filename)
            self.project.load(self.filename)
            self.SetTitle(VERSION + " - " + self.project.name)
            dlg.Destroy()
            self.loadTool()
            self.sb.SetStatusText('opened ' + self.filename)

    def saveFile(self, e=None):
        """Save a robot file from the GUI."""
        if self.filename == "":
            dlg = wx.FileDialog(self, "Choose a file", self.dirname, "", "*.ppr", wx.FD_SAVE)
            if dlg.ShowModal() == wx.ID_OK:
                self.filename = dlg.GetPath()
                self.dirname = dlg.GetDirectory()
                dlg.Destroy()
            else:
                return
        if self.filename[-4:] != ".ppr":
            self.filename = self.filename + ".ppr"
        self.project.saveFile(self.filename)
        self.sb.SetStatusText('saved ' + self.filename)

    def saveFileAs(self, e):
        self.filename = ""
        self.saveFile()

    def export(self, e):
        """Export a pose file for use with Sanguino Library."""
        if self.project.name == "":
            self.sb.SetBackgroundColour('RED')
            self.sb.SetStatusText('please create a project')
            self.timer.Start(20)
            return
        dlg = wx.FileDialog(self, "Choose a file",
                            self.dirname, "", "*.h", wx.FD_SAVE)
        if dlg.ShowModal() == wx.ID_OK:
            self.project.export(dlg.GetPath())
            self.sb.SetStatusText("exported " + dlg.GetPath(), 0)
            dlg.Destroy()

    def findPorts(self):
        """Return a list of serial ports """
        self.ports = []
        for p in list_ports.comports():
            try:
                s = serial.Serial(p.device)
                s.close()
                self.ports.append(p.device)
            except OSError:
                pass

    def doPort(self, e=None):
        """ open a serial port """
        if self.port is None:
            self.findPorts()
        dlg = wx.SingleChoiceDialog(
            self, 'Port (Ex. COM4 or /dev/ttyUSB0)', 'Select Communications Port', self.ports)
        # dlg = PortDialog(self,'Select Communications Port',self.ports)
        if dlg.ShowModal() == wx.ID_OK:
            if self.port is not None:
                self.port.ser.close()
            print("Opening port: " + self.ports[dlg.GetSelection()])
            self.openPort(self.ports[dlg.GetSelection()])
            dlg.Destroy()

    def openPort(self, port, baud=115200, interpolate=True):
        try:
            # TODO: add ability to select type of driver
            self.port = Driver(port, baud, interpolate)
            self.panel.port = self.port
            self.panel.portUpdated()
            self.sb.SetStatusText(port + '@' + str(baud), 1)
        except BaseException:
            self.port = None
            self.sb.SetBackgroundColour('RED')
            self.sb.SetStatusText('Could Not Open Port ' + port, 0)
            self.sb.SetStatusText('not connected', 1)
            self.timer.Start(20)
        return self.port

    def doTest(self, e=None):
        if self.port is not None:
            self.port.execute(253, 25, list())

    def doRelax(self, e=None):
        """ Relax servos so you can pose them. """
        if self.port is not None:
            print("PyPose: relaxing servos...")
            for servo in range(self.project.count):
                self.port.setReg(servo + 1, ax12.P_TORQUE_ENABLE, [0, ])
        else:
            self.sb.SetBackgroundColour('RED')
            self.sb.SetStatusText("No Port Open", 0)
            self.timer.Start(20)

    def doAbout(self, e=None):
        license = """
        This program is free software; you can redistribute it and/or modify
        it under the terms of the GNU General Public License as published by
        the Free Software Foundation; either version 2 of the License, or
        (at your option) any later version.

        This library is distributed in the hope that it will be useful,
        but WITHOUT ANY WARRANTY; without even the implied warranty of
        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
        Lesser General Public License for more details.

        You should have received a copy of the GNU Lesser General Public
        License along with this library; if not, write to the Free Software
        Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA)
        """
        info = wx.adv.AboutDialogInfo()
        info.SetName(VERSION)
        info.SetDescription(
            "A lightweight pose and capture software for the ArbotiX robocontroller")
        info.SetCopyright(
            """
            Copyright (c) 2020 Funtech-Makers.  All right reserved.
            Copyright (c) 2008-2010 Michael E. Ferguson.  All right reserved.
            """)
        info.SetLicense(license)
        info.SetWebSite("http://www.vanadiumlabs.com")
        wx.adv.AboutBox(info)

    def doClose(self, e=None):
        # TODO: edit this to check if we NEED to save...
        if self.project.save:
            dlg = wx.MessageDialog(None, 'Save changes before closing?', '', wx.YES_NO | wx.CANCEL | wx.ICON_QUESTION)
            r = dlg.ShowModal()
            if r == wx.ID_CANCEL:
                e.Veto()
                return
            elif r == wx.ID_YES:
                self.saveFile()
                pass
        self.Destroy()

    def OnTimer(self, e=None):
        self.timeout = self.timeout + 1
        if self.timeout > 50:
            self.sb.SetBackgroundColour(wx.NullColour)
            self.sb.SetStatusText("", 0)
            self.sb.Refresh()
            self.timeout = 0
            self.timer.Stop()

    def do2Col(self, e=None):
        self.columns = 2
        if self.panelIndex == 0:
            self.loadTool()

    def do3Col(self, e=None):
        self.columns = 3
        if isinstance(self.panel, panels[0]):
            self.loadTool()

    def do4Col(self, e=None):
        self.columns = 4
        if isinstance(self.panel, panels[0]):
            self.loadTool()

    def setLiveUpdate(self, e=None):
        if isinstance(self.panel, panels[0]):
            self.panel.live = self.live.IsChecked()


class NewProjectDialog(wx.Dialog):
    """New Project Dialog."""
    def __init__(self, parent, id, title):
        wx.Dialog.__init__(self, parent, id, title, size=(310, 180))

        panel = wx.Panel(self, -1)
        vbox = wx.BoxSizer(wx.VERTICAL)

        wx.StaticBox(panel, -1, 'Project Parameters', (5, 5), (300, 120))
        wx.StaticText(panel, -1, 'Name:', (15, 30))
        self.name = wx.TextCtrl(panel, -1, '', (105, 25))
        wx.StaticText(panel, -1, '# of Servos:', (15, 55))
        self.count = wx.SpinCtrl(panel, -1, '18', (105, 50), min=1, max=30)
        wx.StaticText(panel, -1, 'Resolution:', (15, 80))
        self.resolution = wx.ComboBox(
            panel, -1, '1024', (105, 75), choices=['1024', '4096'])

        hbox = wx.BoxSizer(wx.HORIZONTAL)
        okButton = wx.Button(self, wx.ID_OK, 'Ok', size=(70, 30))
        closeButton = wx.Button(self, wx.ID_CANCEL, 'Close', size=(70, 30))
        hbox.Add(okButton, 1)
        hbox.Add(closeButton, 1, wx.LEFT, 5)

        vbox.Add(panel)
        vbox.Add(hbox, 1, wx.ALIGN_CENTER | wx.TOP | wx.BOTTOM, 10)

        self.SetSizer(vbox)


def main():
    app = wx.App()
    Editor()
    app.MainLoop()


if __name__ == "__main__":
    print("PyPose starting... ")
    main()
