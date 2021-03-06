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
from pypose import project
from .ToolPane import ToolPane


class SeqEditor(ToolPane):
    """ editor for the creation of sequences. """
    BT_MOVE_UP = wx.NewIdRef()
    BT_MOVE_DN = wx.NewIdRef()
    BT_RELAX = wx.NewIdRef()
    BT_RUN = wx.NewIdRef()
    BT_LOOP = wx.NewIdRef()
    BT_HALT = wx.NewIdRef()
    BT_SEQ_ADD = wx.NewIdRef()
    BT_SEQ_REM = wx.NewIdRef()
    BT_TRAN_ADD = wx.NewIdRef()
    BT_TRAN_REM = wx.NewIdRef()

    ID_SEQ_BOX = wx.NewIdRef()
    ID_TRAN_BOX = wx.NewIdRef()
    ID_TRAN_POSE = wx.NewIdRef()
    ID_TRAN_TIME = wx.NewIdRef()

    NAME = "Sequence editor"
    STATUS = "please create or select a sequence to edit..."

    def __init__(self, parent, port=None):
        ToolPane.__init__(self, parent, port)
        self.curseq = ""
        self.curtran = -1

        sizer = wx.GridBagSizer(10, 10)

        # sequence editor, goes in a box:
        temp = wx.StaticBox(self, -1, 'edit sequence')
        temp.SetFont(wx.Font(10, wx.DEFAULT, wx.NORMAL, wx.BOLD))
        editBox = wx.StaticBoxSizer(temp, orient=wx.VERTICAL)
        seqEditSizer = wx.GridBagSizer(5, 5)

        # transitions list
        temp = wx.StaticText(self, -1, "transitions:")
        temp.SetFont(wx.Font(10, wx.DEFAULT, wx.NORMAL, wx.BOLD))
        seqEditSizer.Add(temp, (0, 0), wx.GBSpan(1, 1), wx.TOP, 10)
        self.tranbox = wx.ListBox(self, self.ID_TRAN_BOX)
        seqEditSizer.Add(self.tranbox, (1, 0),
                         wx.GBSpan(5, 1), wx.EXPAND | wx.ALL)
        # and add/remove
        hbox = wx.BoxSizer(wx.HORIZONTAL)
        hbox.Add(wx.Button(self, self.BT_TRAN_ADD, 'new'))
        hbox.Add(wx.Button(self, self.BT_TRAN_REM, 'delete'))
        seqEditSizer.Add(hbox, (6, 0), wx.GBSpan(1, 1), wx.ALIGN_CENTER)

        # pose name & delta-T
        seqEditSizer.Add(wx.StaticText(self, -1, "pose:"), (1, 1))
        self.tranPose = wx.ComboBox(self, self.ID_TRAN_POSE, choices=[*self.parent.project.poses])
        seqEditSizer.Add(self.tranPose, (1, 2))
        seqEditSizer.Add(wx.StaticText(self, -1, "delta-T:"), (2, 1))
        self.tranTime = wx.SpinCtrl(
            self, self.ID_TRAN_TIME, '1000', min=1, max=5000)
        seqEditSizer.Add(self.tranTime, (2, 2))
        # buttons to move transition up/down
        hbox = wx.BoxSizer(wx.HORIZONTAL)
        hbox.Add(wx.Button(self, self.BT_MOVE_UP, 'move up'))
        hbox.Add(wx.Button(self, self.BT_MOVE_DN, 'move down'))
        seqEditSizer.Add(hbox, (4, 1), wx.GBSpan(1, 2), wx.ALIGN_CENTER | wx.BOTTOM, 10)
        # grid it
        editBox.Add(seqEditSizer)
        sizer.Add(editBox, (0, 0), wx.GBSpan(1, 1), wx.EXPAND)

        # list of sequences
        self.seqbox = wx.ListBox(
            self, self.ID_SEQ_BOX, choices=[*self.parent.project.sequences])
        sizer.Add(self.seqbox, (0, 1), wx.GBSpan(1, 1), wx.EXPAND)
        # and add/remove
        hbox = wx.BoxSizer(wx.HORIZONTAL)
        hbox.Add(wx.Button(self, self.BT_SEQ_ADD, 'add'))
        hbox.Add(wx.Button(self, self.BT_SEQ_REM, 'remove'))
        sizer.Add(hbox, (1, 1), wx.GBSpan(1, 1), wx.ALIGN_CENTER)

        # toolbar
        toolbar = wx.Panel(self, -1)
        toolbarsizer = wx.BoxSizer(wx.HORIZONTAL)
        toolbarsizer.Add(wx.Button(toolbar, self.BT_RELAX, 'relax'), 1)
        toolbarsizer.Add(wx.Button(toolbar, self.BT_RUN, 'run'), 1)
        toolbarsizer.Add(wx.Button(toolbar, self.BT_LOOP, 'loop'), 1)
        toolbarsizer.Add(wx.Button(toolbar, self.BT_HALT, 'halt'), 1)
        toolbar.SetSizer(toolbarsizer)
        sizer.Add(toolbar, (1, 0), wx.GBSpan(1, 1), wx.ALIGN_CENTER)

        self.SetSizerAndFit(sizer)

        self.Bind(wx.EVT_BUTTON, self.parent.doRelax, self.BT_RELAX)
        self.Bind(wx.EVT_BUTTON, self.runSeq, self.BT_RUN)
        self.Bind(wx.EVT_BUTTON, self.runSeq, self.BT_LOOP)
        self.Bind(wx.EVT_BUTTON, self.haltSeq, self.BT_HALT)
        self.Bind(wx.EVT_BUTTON, self.addSeq, self.BT_SEQ_ADD)
        self.Bind(wx.EVT_BUTTON, self.remSeq, self.BT_SEQ_REM)
        self.Bind(wx.EVT_LISTBOX, self.doSeq, self.ID_SEQ_BOX)
        self.Bind(wx.EVT_BUTTON, self.moveUp, self.BT_MOVE_UP)
        self.Bind(wx.EVT_BUTTON, self.moveDn, self.BT_MOVE_DN)
        self.Bind(wx.EVT_BUTTON, self.addTran, self.BT_TRAN_ADD)
        self.Bind(wx.EVT_BUTTON, self.remTran, self.BT_TRAN_REM)
        self.Bind(wx.EVT_LISTBOX, self.doTran, self.ID_TRAN_BOX)
        self.Bind(wx.EVT_COMBOBOX, self.updateTran, self.ID_TRAN_POSE)
        self.Bind(wx.EVT_SPINCTRL, self.updateTran, self.ID_TRAN_TIME)

    def save(self):
        if self.curseq != "":
            self.parent.project.sequences[self.curseq] = project.Sequence()
            for i in range(self.tranbox.GetCount()):
                self.parent.project.sequences[self.curseq].append(
                    self.tranbox.GetString(i).replace(",", "|"))
            self.parent.project.save = True

    def doSeq(self, e=None):
        """Save previous sequence changes, load a sequence into the editor."""
        if e.IsSelection():
            self.save()
            self.curseq = str(e.GetString())
            self.curtran = -1
            for i in range(self.tranbox.GetCount()):
                # TODO: There has got to be a better way to do this??
                self.tranbox.Delete(0)
            for transition in self.parent.project.sequences[self.curseq]:
                self.tranbox.Append(transition.replace("|", ","))
            self.tranPose.SetValue("")
            self.tranTime.SetValue(500)
            self.parent.sb.SetStatusText(
                'now editing sequence: ' + self.curseq)

    def addSeq(self, e=None):
        """ create a new sequence. """
        if self.parent.project.name != "":
            dlg = wx.TextEntryDialog(
                self, 'Sequence Name:', 'New Sequence Settings')
            dlg.SetValue("")
            if dlg.ShowModal() == wx.ID_OK:
                self.seqbox.Append(dlg.GetValue())
                self.parent.project.sequences[dlg.GetValue(
                )] = project.Sequence("")
                dlg.Destroy()
                self.parent.project.save = True
        else:
            dlg = wx.MessageDialog(
                self, 'Please create a new robot first.', 'Error', wx.OK | wx.ICON_EXCLAMATION)
            dlg.ShowModal()
            dlg.Destroy()

    def remSeq(self, e=None):
        """ remove a sequence. """
        if self.curseq != "":
            dlg = wx.MessageDialog(self, 'Are you sure you want to delete this sequence?',
                                   'Confirm', wx.OK | wx.CANCEL | wx.ICON_EXCLAMATION)
            if dlg.ShowModal() == wx.ID_OK:
                # this order is VERY important!
                v = self.seqbox.FindString(self.curseq)
                del self.parent.project.sequences[self.curseq]
                self.seqbox.Delete(v)
                self.curseq = ""
                dlg.Destroy()
                self.parent.project.save = True

    def doTran(self, e=None):
        """Load a transition into the editor."""
        if e.IsSelection():
            if self.curseq != "":
                self.curtran = e.GetInt()
                v = str(e.GetString())
                self.tranPose.SetValue(v[0:v.find(",")])
                self.tranTime.SetValue(int(v[v.find(",") + 1:]))
                self.parent.project.save = True

    def addTran(self, e=None):
        """Create a new transtion in this sequence."""
        if self.curseq != "":
            if self.curtran != -1:
                self.tranbox.Insert("none,500", self.curtran + 1)
            else:
                self.tranbox.Append("none,500")
            self.parent.project.save = True

    def remTran(self, e=None):
        """ remove a sequence. """
        if self.curseq != "" and self.curtran != -1:
            dlg = wx.MessageDialog(self, 'Are you sure you want to delete this transition?',
                                   'Confirm', wx.OK | wx.CANCEL | wx.ICON_EXCLAMATION)
            if dlg.ShowModal() == wx.ID_OK:
                self.tranbox.Delete(self.curtran)
                self.curtran = -1
                self.tranPose.SetValue("")
                self.tranTime.SetValue(500)
                dlg.Destroy()
                self.parent.project.save = True

    def moveUp(self, e=None):
        if self.curtran > 0:
            self.tranbox.Delete(self.curtran)
            self.curtran = self.curtran - 1
            self.tranbox.Insert(self.tranPose.GetValue(
            ) + "," + str(self.tranTime.GetValue()), self.curtran)
            self.tranbox.SetSelection(self.curtran)
            self.parent.project.save = True

    def moveDn(self, e=None):
        if self.curtran < self.tranbox.GetCount() - 1:
            self.tranbox.Delete(self.curtran)
            self.curtran = self.curtran + 1
            self.tranbox.Insert(self.tranPose.GetValue(
            ) + "," + str(self.tranTime.GetValue()), self.curtran)
            self.tranbox.SetSelection(self.curtran)
            self.parent.project.save = True

    def updateTran(self, e=None):
        if self.curtran != -1:
            self.tranbox.Delete(self.curtran)
            self.tranbox.Insert(self.tranPose.GetValue() + "," + str(self.tranTime.GetValue()), self.curtran)
            print("Updated:", self.tranPose.GetValue(), ",", str(self.tranTime.GetValue()), self.curtran)
            self.tranbox.SetSelection(self.curtran)
            self.parent.project.save = True

    def runSeq(self, e=None):
        """ download poses, seqeunce, and send. """
        self.save()  # save sequence
        if self.port is not None:
            if self.curseq != "":
                print("Run sequence...")
                # key = pose name, val = index, download them after we build a transition list
                poseDL = dict()
                tranDL = list()     # list of bytes to download
                for t in self.parent.project.sequences[self.curseq]:
                    p = t[0:t.find("|")]                    # pose name
                    dt = int(t[t.find("|") + 1:])             # delta-T
                    if p not in poseDL.keys():
                        poseDL[p] = len(poseDL.keys())      # get ix for pose
                    # create transition values to download
                    tranDL.append(poseDL[p])                # ix of pose
                    # time is an int (16-bytes)
                    tranDL.append(dt % 256)
                    tranDL.append(dt >> 8)
                tranDL.append(255)      # notice to stop
                tranDL.append(0)        # time is irrelevant on stop
                tranDL.append(0)
                # set pose size -- IMPORTANT!
                print("Setting pose size at " + str(self.parent.project.count))
                self.port.execute(253, 7, [self.parent.project.count])
                # send poses
                for p in poseDL.keys():
                    print("Sending pose " + str(p) + " to position " + str(poseDL[p]))
                    self.port.execute(
                        253, 8, [poseDL[p]] + project.extract(self.parent.project.poses[p]))
                print("Sending sequence: " + str(tranDL))
                # send sequence and play
                self.port.execute(253, 9, tranDL)
                # run or loop?
                if e.GetId() == self.BT_LOOP:
                    self.port.execute(253, 11, list())
                else:
                    self.port.execute(253, 10, list())
                self.parent.sb.SetStatusText('Playing Sequence: ' + self.curseq)
            else:
                self.parent.sb.SetBackgroundColour('RED')
                self.parent.sb.SetStatusText("Select a Sequence", 0)
                self.parent.timer.Start(20)
        else:
            self.parent.sb.SetBackgroundColour('RED')
            self.parent.sb.SetStatusText("No Port Open", 0)
            self.parent.timer.Start(20)

    def haltSeq(self, e=None):
        """ send halt message ("H") """
        if self.port is not None:
            print("Halt sequence...")
            self.port.ser.write(b"H")
        else:
            self.parent.sb.SetBackgroundColour('RED')
            self.parent.sb.SetStatusText("No Port Open", 0)
            self.parent.timer.Start(20)
