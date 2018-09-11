#!/usr/bin/env python

import sys
from lfd import *
from PyQt4 import QtGui, QtCore
import threading
import time
import rospy
import rosparam
import yaml
import os
from time import time

class LfDGui(QtGui.QMainWindow):

	def __init__(self):
		super(LfDGui, self).__init__()
		# lfd instance
		self.lfd = LfD()

		# Thread for execution
		self.executingThread = None

		# Callback function for the button in actions
		self.button_cb = None

		# Set up the basic frame
		self.setGeometry(50, 50, 500, 600)
		self.setFixedSize(500, 600)

		# Set up the status bar
		self.statusBar = QtGui.QStatusBar()
		self.statusBar.showMessage('version 0.0')
		self.setStatusBar(self.statusBar)

		# Main menu actions
		exitAction = QtGui.QAction("&Exit", self)
		exitAction.setShortcut("Ctrl+Q")
		exitAction.setStatusTip('Exit the GUI')
		exitAction.triggered.connect(self.closeEvent)

		# Main Menu
		mainMenu = self.menuBar()
		# File Menu
		fileMenu = mainMenu.addMenu('&Exit')
		fileMenu.addAction(exitAction)

		# Add a timer for displaying state
		self.state_timer = QtCore.QTimer(self)
		self.state_timer.setInterval(42) # Meaning of life or 24fps

		# Show the home interface
		self.display_home()

	def display_home(self):
		# Home layout widget
		home_layout = Home()

		# Set up button connections
		home_layout.btn_d.clicked.connect(self.display_action)
		home_layout.btn_l.clicked.connect(self.learn_cb)
		home_layout.btn_e_dt.clicked.connect(self.execute_cb)
		home_layout.btn_e_sim.clicked.connect(self.execute_cb)
		home_layout.btn_e_sop.clicked.connect(self.execute_cb)
		home_layout.btn_e_esp.clicked.connect(self.execute_cb)
		home_layout.btn_e.clicked.connect(self.execute_cb)
		home_layout.btn_r.clicked.connect(self.display_action)
		home_layout.btn_lm.clicked.connect(self.loadmodel_cb)
		home_layout.btn_wm.clicked.connect(self.writemodel_cb)
		home_layout.btn_ld.clicked.connect(self.load_demos_cb)
		home_layout.btn_wd.clicked.connect(self.write_demos_cb)
		home_layout.btn_clear_demos.clicked.connect(self.clear_demos_cb)
		home_layout.btn_u.clicked.connect(self.undo_cb)
		home_layout.btn_rd.clicked.connect(self.redo_cb)
		home_layout.btn_load.clicked.connect(self.load_config_file)
		home_layout.btn_quit.clicked.connect(self.closeEvent)

		self.state_timer.stop()

		# Show the layout
		self.setGeometry(200, 200, 750, 500)
		self.setWindowTitle("Learning from Demonstration")
		self.setWindowIcon(QtGui.QIcon('lfd_logo.png'))
		self.setCentralWidget(home_layout)
		self.show()

		# Turn off trees
		self.lfd.stop_bt()
		self.lfd.stop_perception_tree()

	def display_action(self):
		'''
		TODO: All button setting should be done
		in the constructor of ActionInterface, not here
		'''

		# Action interface layout
		self.action_interface_layout = ActionInterface()
		self.state_timer.start()

		# Choose callback function
		if self.sender().objectName() == "Demonstrate":
			self.button_cb = self.demonstrate_cb
		elif self.sender().objectName() == "Rc":
			self.button_cb = self.rc_cb
		elif self.sender().objectName() == "add_tts_behavior":
			pass
		else:
			self.button_cb = None

		if self.button_cb == self.demonstrate_cb:
			self.action_interface_layout.meau_label.setText("Demonstrate - " + "Actions:")
		elif self.button_cb == self.rc_cb:
			self.action_interface_layout.menu_label.setText("Rc - " + "Actions:")

		# Action buttons
		for key, name in self.lfd.action_names.iteritems():
			button = QtGui.QPushButton(name)
			button.setObjectName(name)
			button.clicked.connect(self.button_cb)
			button.setFont(self.action_interface_layout.newFont)
			self.action_interface_layout.but_layout.addWidget(button)

		self.action_interface_layout.but_layout.addStretch()

		# Label
		self.action_interface_layout.back_label = QtGui.QLabel()
		self.action_interface_layout.back_label.setText("Options:")
		self.action_interface_layout.back_label.setFont(self.action_interface_layout.menu_font)

		self.action_interface_layout.but_layout.addWidget(self.action_interface_layout.back_label)

		# Go Back Button
		self.action_interface_layout.back_btn = QtGui.QPushButton("Go Back")
		self.action_interface_layout.back_btn.setFont(self.action_interface_layout.newFont)
		self.action_interface_layout.but_layout.addWidget(self.action_interface_layout.back_btn)

		# Set up go back button connections
		self.action_interface_layout.back_btn.clicked.connect(self.display_home)

		# Button for add TTS behaviors
		self.action_interface_layout.tts_btn = QtGui.QPushButton("Add TTS Behavior")
		self.action_interface_layout.tts_btn.setFont(self.action_interface_layout.newFont)
		self.action_interface_layout.but_layout.addWidget(self.action_interface_layout.tts_btn)

		# Set up add tts button connections
		self.action_interface_layout.tts_btn.clicked.connect(self.display_tts)

		# Update state
		self.action_interface_layout.state_label.setText(self.lfd.state_to_str(self.lfd.get_state()))
		self.state_timer.timeout.connect(self.update_state)

		# Show the layout
		self.setCentralWidget(self.action_interface_layout)
		self.show()

		# Restart perception tree
		self.lfd.stop_perception_tree()
		self.lfd.start_perception_tree()

	@QtCore.pyqtSlot()
	def update_state(self):
		self.action_interface_layout.state_label.setText(self.lfd.state_to_str(self.lfd.get_state()))

	def display_exec(self):
		# Execute interface layout
		exec_interface = ExecuteInterface()

		# Add connections
		exec_interface.interrupt_btn.clicked.connect(self.interrupt_exec)

		# Show the layout
		self.setCentralWidget(exec_interface)
		self.show()

	def display_tts(self):
		# Add TTS interface layout
		tts_interface = TTSInterface()

		# Add connectnios
		tts_interface.text_field.returnPressed.connect(self.add_tts_behavior)
		tts_interface.back_btn.clicked.connect(self.display_action)

		# Show the layout
		self.setCentralWidget(tts_interface)
		self.show()

	def add_tts_behavior(self):
		line_edit = self.sender()
		tts_text = str(line_edit.text())
		new_name = 'say ' + tts_text
		new_key = len(self.lfd.actions)
		# Add new action
		self.lfd.actions[new_key] = Action(new_name, BuildTTSBehavior, tts_text)
		self.lfd.action_names[new_key] = new_name
		self.lfd.action_indices[new_name] = new_key
		# Clear the editor and output the prompt
		line_edit.clear()
		QtGui.QMessageBox.information(self, 'Success', 'Success in adding TTS behavior!')

	def add_nav_behavior(self):
		new_key = len(self.lfd.actions)
		# Add nav actions
		self.lfd.actions[new_key] = Action('nav_to_bag', BuildNavToBagBehavior)
		self.lfd.action_names[new_key] = 'nav_to_bag'
		self.lfd.action_indices['nav_to_bag'] = new_key

		new_key += 1
		self.lfd.actions[new_key] = Action('nav_to_items', BuildNavToItemsBehavior)
		self.lfd.action_names[new_key] = 'nav_to_items'
		self.lfd.action_indices['nav_to_items'] = new_key

		new_key += 1
		self.lfd.actions[new_key] = Action('nav_to_home', BuildNavToHomeBehavior)
		self.lfd.action_names[new_key] = 'nav_to_home'
		self.lfd.action_indices['nav_to_home'] = new_key

		new_key += 1
		self.lfd.actions[new_key] = Action('nav_to_item1', BuildNavToItem1Behavior)
		self.lfd.action_names[new_key] = 'nav_to_item1'
		self.lfd.action_indices['nav_to_item1'] = new_key

		new_key += 1
		self.lfd.actions[new_key] = Action('nav_to_item2', BuildNavToItem2Behavior)
		self.lfd.action_names[new_key] = 'nav_to_item2'
		self.lfd.action_indices['nav_to_item2'] = new_key

		new_key += 1
		self.lfd.actions[new_key] = Action('nav_to_item3', BuildNavToItem3Behavior)
		self.lfd.action_names[new_key] = 'nav_to_item3'
		self.lfd.action_indices['nav_to_item3'] = new_key

	def load_config_file(self):
		# Load the configure file through cmd line
		name = QtGui.QFileDialog.getOpenFileName(self, 'Open File', filter='*.yaml')
		yaml_file = open(name[0], 'r')
		yaml_data = yaml.load(yaml_file)
		yaml_file.close()
		rosparam.upload_params('/', yaml_data)
		# Check whether the nav actions have been created
		# if 'nav_to_bag' not in self.lfd.action_indices or 'nav_to_items' not in self.lfd.action_indices:
		# 	self.add_nav_behavior()
		QtGui.QMessageBox.information(self, 'Success', 'Success in loading configure file for nav behavior!')

	def redo_cb(self):
		if self.lfd.redo_states is not None and len(self.lfd.redo_states) != 0:
			# print self.demo_states
			# print self.redo_states
			# print self.redo_states[-1]
			self.lfd.demo_states = numpy.append(self.lfd.demo_states, [self.lfd.redo_states[-1]], axis=0)
			self.lfd.demo_actions = numpy.append(self.lfd.demo_actions, [self.lfd.redo_actions[-1]])
			# self.last_actions.append(self.redo_last_actions[-1])
			self.redo_states = self.redo_states[:-1]
			self.redo_actions = self.redo_actions[:-1]
			# self.redo_last_actions = self.redo_last_actions[:-1]
			QtGui.QMessageBox.information(self, 'Success', 'Success in redoing the action!')
		else:
			print("No undone demonstration to redo.")
			QtGui.QMessageBox.information(self, 'Failure', 'No undone demonstration to redo!')

	def undo_cb(self):
		if self.lfd.demo_states is not None and len(self.lfd.demo_states) != 0:
			if self.lfd.redo_states is not None and len(self.lfd.redo_states) != 0:
				self.lfd.redo_states = numpy.append(self.lfd.redo_states, [self.lfd.demo_states[-1]], axis=0)
				self.lfd.redo_actions = numpy.append(self.lfd.redo_actions, [self.lfd.demo_actions[-1]])
				# self.redo_last_actions = numpy.append(self.redo_last_actions, [self.last_actions[-1]])
			else:
				self.lfd.redo_states = [self.lfd.demo_states[-1]]
				self.lfd.redo_actions = [self.lfd.demo_actions[-1]]
				# self.redo_last_actions = [self.last_actions[-1]]
			self.lfd.demo_states = self.lfd.demo_states[:-1]
			self.lfd.demo_actions = self.lfd.demo_actions[:-1]
			# self.last_actions.pop()
			QtGui.QMessageBox.information(self, 'Success', 'Success in undoing the action!')
		else:
			print "No demonstration to undo."
			QtGui.QMessageBox.information(self, 'Failure', 'Nothing to undo!')


	def writemodel_cb(self):
		name = QtGui.QFileDialog.getSaveFileName(self, 'Save File')
		pickle.dump((self.lfd.clf, self.lfd.retained_feats_names, self.lfd.demo_states, self.lfd.demo_actions), open(name[0], 'wb'))
		QtGui.QMessageBox.information(self, 'Success', 'Success in saving the model!')

	def loadmodel_cb(self):
		name = QtGui.QFileDialog.getOpenFileName(self, 'Open File')
		(self.lfd.clf, self.lfd.retained_feats_names, self.lfd.demo_states, self.lfd.demo_actions) = pickle.load(open(name[0], 'rb'))
		self.lfd.render_model()
		QtGui.QMessageBox.information(self, 'Success', 'Success in loading the model!')

	def write_demos_cb(self):
		demo_file = QtGui.QFileDialog.getSaveFileName(self, 'Save File')[0]
		self.write_demos(demo_file)
		QtGui.QMessageBox.information(self, 'Success', 'Success in saving the demos!')

	def write_demos(self, demo_file):
		pickle.dump((self.lfd.demo_states, self.lfd.demo_actions), open(demo_file, 'wb'))

	def load_demos_cb(self):
		name = QtGui.QFileDialog.getOpenFileName(self, 'Open File')
		(self.lfd.demo_states, self.lfd.demo_actions) = pickle.load(open(name[0], 'rb'))
		QtGui.QMessageBox.information(self, 'Success', 'Success in loading the demos!')

	def clear_demos_cb(self):
		self.lfd.demo_states = None
		self.lfd.demo_actions = None
		QtGui.QMessageBox.information(self, 'Success', 'Success in clearing the demos!')

	def rc_cb(self):
		action_name = self.sender().objectName()
		self.lfd.run_action(action_name)
		QtGui.QMessageBox.information(self, 'Success', 'Success in performing ' + action_name +'!')

	def execute_cb(self):

		if self.lfd.clf.named_steps['classification'] != None:
			execute_flag = True
		else:
			execute_flag = False
		if execute_flag == False:
			QtGui.QMessageBox.warning(self,"Error",
                             self.tr("The robot haven't learned anything yet!"))
		else:
			# Shut down the currently active executing thread
			if self.executingThread != None:
				self.executingThread.interrupt_flag = True
				self.executingThread = None
			# Show the interface
			self.display_exec()
			# Create a new thread for ticking the tree
			mode = self.sender().objectName()
			if mode == 'Execute_dt':
				self.lfd.start_perception_tree()
			self.executingThread = execThread('execThread', self.lfd, mode)
			self.executingThread.start()

	def interrupt_exec(self):
		if self.lfd.tree != None:
			self.lfd.tree.interrupt()
		# Stop the running thread
		self.executingThread.interrupt_flag = True
		self.executingThread = None
		self.lfd.stop_perception_tree()
		QtGui.QMessageBox.information(self, 'Success', 'Stop the execution!')
		self.display_home()

	def learn_cb(self):
		self.lfd.learn(self.lfd.demo_states, self.lfd.demo_actions)
		QtGui.QMessageBox.information(self, 'Success', 'The robot has finished the learning!')

	def demonstrate_cb(self):
		world_state = self.lfd.get_state()
		print('World state is')
		self.lfd.print_state(world_state)
		action_name = str(self.sender().objectName())
		user_input = numpy.array([[action_name]])
		self.lfd.run_action(action_name)
		# Save the state action combo
		if self.lfd.demo_states is not None:
			self.lfd.demo_states = numpy.append(self.lfd.demo_states, world_state, axis=0)
			self.lfd.demo_actions = numpy.append(self.lfd.demo_actions, user_input)
		else:
			self.lfd.demo_states = world_state
			self.lfd.demo_actions = user_input

		# Write to a temp file
		self.write_demos('media/current.demos')

		QtGui.QMessageBox.information(self, 'Success', 'Success in performing ' + action_name +'!')

	def closeEvent(self, event):
		choice = QtGui.QMessageBox.question(self, 'Exit Prompt',
											"Do you want to close LfD Gui?",
											QtGui.QMessageBox.Yes | QtGui.QMessageBox.No)
		if choice == QtGui.QMessageBox.Yes:
			# Shut down the active executing thread
			if self.executingThread != None:
				self.executingThread.interrupt_flag = True
				self.executingThread = None
				print("Shut down the active executing thread.")
			print("LfD Gui exits.")
			sys.exit()
		else:
			pass

class Home(QtGui.QWidget):

	def __init__(self):
		super(Home, self).__init__()

		# Font
		self.newFont = QtGui.QFont("Helvetica", 16, QtGui.QFont.Bold)

		# Main Layout
		self.vlayout = QtGui.QVBoxLayout()

		# Label
		self.menu_font = QtGui.QFont()
		self.menu_font.setPointSize(24)
		self.menu_font.setBold(True)
		self.menu_font.setWeight(75)
		self.menu_label = QtGui.QLabel()
		self.menu_label.setText("Menu:")
		self.menu_label.setFont(self.menu_font)

		self.vlayout.addWidget(self.menu_label)
		self.vlayout.addStretch()

		# Buttons for users
		self.btn_d = QtGui.QPushButton("Demonstrate")
		self.btn_d.resize(self.btn_d.minimumSizeHint())
		self.btn_d.setObjectName("Demonstrate")
		self.btn_d.setFont(self.newFont)
		self.btn_d.move(0, 100)

		self.btn_l = QtGui.QPushButton("Learn")
		self.btn_l.resize(self.btn_l.minimumSizeHint())
		self.btn_l.setObjectName("Learn")
		self.btn_l.setFont(self.newFont)
		self.btn_l.move(0, 100)

		# Button for decision tree method
		self.btn_e_dt = QtGui.QPushButton("Execute_dt")
		self.btn_e_dt.resize(self.btn_e_dt.minimumSizeHint())
		self.btn_e_dt.setObjectName("Execute_dt")
		self.btn_e_dt.setFont(self.newFont)
		self.btn_e_dt.move(0, 100)

		# Button for simple behavior tree method
		self.btn_e_sim = QtGui.QPushButton("Execute_naive_algo")
		self.btn_e_sim.resize(self.btn_e_sim.minimumSizeHint())
		self.btn_e_sim.setObjectName("Execute_naive_algo")
		self.btn_e_sim.setFont(self.newFont)
		self.btn_e_sim.move(0, 100)

		# Button for SOP behavior tree method
		self.btn_e_sop = QtGui.QPushButton("Execute_SOP")
		self.btn_e_sop.resize(self.btn_e_sop.minimumSizeHint())
		self.btn_e_sop.setObjectName("Execute_SOP")
		self.btn_e_sop.setFont(self.newFont)
		self.btn_e_sop.move(0, 100)

		# Button for CDNF behavior tree method
		self.btn_e = QtGui.QPushButton("Execute_CDNF")
		self.btn_e.resize(self.btn_e.minimumSizeHint())
		self.btn_e.setObjectName("Execute_CDNF")
		self.btn_e.setFont(self.newFont)
		self.btn_e.move(0, 100)

		# Button for Espresso behavior tree method
		self.btn_e_esp = QtGui.QPushButton("Execute_BTEspresso")
		self.btn_e_esp.resize(self.btn_e_esp.minimumSizeHint())
		self.btn_e_esp.setObjectName("Execute_BTEspresso")
		self.btn_e_esp.setFont(self.newFont)
		self.btn_e_esp.move(0, 100)

		self.btn_r = QtGui.QPushButton("Rc")
		self.btn_r.resize(self.btn_r.minimumSizeHint())
		self.btn_r.setObjectName("Rc")
		self.btn_r.setFont(self.newFont)
		self.btn_r.move(0, 100)

		self.btn_lm = QtGui.QPushButton("Load model")
		self.btn_lm.resize(self.btn_lm.minimumSizeHint())
		self.btn_lm.setObjectName("Load model")
		self.btn_lm.setFont(self.newFont)
		self.btn_lm.move(0, 100)

		self.btn_wm = QtGui.QPushButton("Write model")
		self.btn_wm.resize(self.btn_wm.minimumSizeHint())
		self.btn_wm.setObjectName("Write model")
		self.btn_wm.setFont(self.newFont)
		self.btn_wm.move(0, 100)

		self.btn_ld = QtGui.QPushButton("Load demos")
		self.btn_ld.resize(self.btn_lm.minimumSizeHint())
		self.btn_ld.setObjectName("Load demos")
		self.btn_ld.setFont(self.newFont)
		self.btn_ld.move(0, 100)

		self.btn_wd = QtGui.QPushButton("Write demos")
		self.btn_wd.resize(self.btn_wm.minimumSizeHint())
		self.btn_wd.setObjectName("Write demos")
		self.btn_wd.setFont(self.newFont)
		self.btn_wd.move(0, 100)

		self.btn_clear_demos = QtGui.QPushButton("Clear demos")
		self.btn_clear_demos.resize(self.btn_wm.minimumSizeHint())
		self.btn_clear_demos.setObjectName("Clear demos")
		self.btn_clear_demos.setFont(self.newFont)
		self.btn_clear_demos.move(0, 100)

		self.btn_u = QtGui.QPushButton("Undo last demo")
		self.btn_u.resize(self.btn_u.minimumSizeHint())
		self.btn_u.setObjectName("Undo last demo")
		self.btn_u.setFont(self.newFont)
		self.btn_u.move(0, 100)

		self.btn_rd = QtGui.QPushButton("Redo last undone")
		self.btn_rd.resize(self.btn_rd.minimumSizeHint())
		self.btn_rd.setObjectName("Redo last undone")
		self.btn_rd.setFont(self.newFont)
		self.btn_rd.move(0, 100)

		self.vlayout.addWidget(self.btn_d)
		self.vlayout.addWidget(self.btn_l)
		self.vlayout.addWidget(self.btn_e_dt)
		self.vlayout.addWidget(self.btn_e_sim)
		self.vlayout.addWidget(self.btn_e_sop)
		self.vlayout.addWidget(self.btn_e)
		self.vlayout.addWidget(self.btn_e_esp)
		self.vlayout.addWidget(self.btn_r)
		self.vlayout.addWidget(self.btn_lm)
		self.vlayout.addWidget(self.btn_wm)
		self.vlayout.addWidget(self.btn_ld)
		self.vlayout.addWidget(self.btn_wd)
		self.vlayout.addWidget(self.btn_clear_demos)
		self.vlayout.addWidget(self.btn_u)
		self.vlayout.addWidget(self.btn_rd)

		# Other options
		self.options_label = QtGui.QLabel()
		self.options_label.setText("Other options:")
		self.options_label.setFont(self.menu_font)

		self.vlayout.addWidget(self.options_label)
		self.vlayout.addStretch()

		self.btn_load = QtGui.QPushButton("Load configure file")
		self.btn_load.resize(self.btn_load.minimumSizeHint())
		self.btn_load.setFont(self.newFont)
		self.btn_load.move(0, 100)

		self.btn_quit = QtGui.QPushButton("Quit")
		self.btn_quit.resize(self.btn_quit.minimumSizeHint())
		self.btn_quit.setFont(self.newFont)
		self.btn_quit.move(0, 100)

		self.vlayout.addWidget(self.btn_load)
		self.vlayout.addWidget(self.btn_quit)

		# Set up the layout
		self.setLayout(self.vlayout)

class ActionInterface(QtGui.QWidget):

	def __init__(self):
		super(ActionInterface, self).__init__()
		# Font
		self.newFont = QtGui.QFont("Helvetica", 16, QtGui.QFont.Bold)

		# Layout
		self.vlayout = QtGui.QVBoxLayout()

		# Label
		self.menu_font = QtGui.QFont()
		self.menu_font.setPointSize(24)
		self.menu_font.setBold(True)
		self.menu_font.setWeight(75)
		self.menu_label = QtGui.QLabel()
		self.menu_label.setText("Actions:")
		self.menu_label.setFont(self.menu_font)

		self.vlayout.addWidget(self.menu_label)
		# self.vlayout.addStretch()

		self.hlayout = QtGui.QHBoxLayout()
		# self.state_scroll = QtGui.QScrollArea()
		self.state_label = QtGui.QLabel()
		# self.state_scroll.setWidget(self.state_label)
		self.but_layout = QtGui.QVBoxLayout()
		# self.hlayout.addWidget(self.state_scroll)
		self.hlayout.addWidget(self.state_label)
		self.hlayout.addLayout(self.but_layout)

		self.vlayout.addLayout(self.hlayout)

		self.setLayout(self.vlayout)

class ExecuteInterface(QtGui.QWidget):

	def __init__(self):
		super(ExecuteInterface, self).__init__()
		# Font
		self.newFont = QtGui.QFont("Helvetica", 16, QtGui.QFont.Bold)

		# Layout
		self.vlayout = QtGui.QVBoxLayout()

		# Label
		self.menu_font = QtGui.QFont()
		self.menu_font.setPointSize(24)
		self.menu_font.setBold(True)
		self.menu_font.setWeight(75)
		self.menu_label = QtGui.QLabel()
		self.menu_label.setText("Options:")
		self.menu_label.setFont(self.menu_font)

		self.vlayout.addWidget(self.menu_label)
		self.vlayout.addStretch()

		# Buttons
		self.interrupt_btn = QtGui.QPushButton('Stop Execution and Go Back')
		self.interrupt_btn.setFont(self.newFont)
		self.interrupt_btn.resize(self.interrupt_btn.sizeHint())
		self.vlayout.addWidget(self.interrupt_btn)

		self.setLayout(self.vlayout)

class TTSInterface(QtGui.QWidget):

	def __init__(self):
		super(TTSInterface, self).__init__()

		# Font
		self.newFont = QtGui.QFont("Helvetica", 16, QtGui.QFont.Bold)

		# Layout
		self.vlayout = QtGui.QVBoxLayout()

		# Label
		self.menu_font = QtGui.QFont()
		self.menu_font.setPointSize(24)
		self.menu_font.setBold(True)
		self.menu_font.setWeight(75)
		self.menu_label = QtGui.QLabel()
		self.menu_label.setText("Add Text:")
		self.menu_label.setFont(self.menu_font)

		self.vlayout.addWidget(self.menu_label)
		self.vlayout.addStretch()

		# Add prompt
		self.prompt_font = QtGui.QFont()
		self.prompt_font.setPointSize(14)
		self.prompt_font.setBold(False)
		self.prompt = QtGui.QLabel()
		self.prompt.setText("Press Enter after inputing the text")
		self.prompt.setFont(self.prompt_font)

		self.vlayout.addWidget(self.prompt)
		self.vlayout.addStretch()

		# Text Field
		self.text_field = QtGui.QLineEdit()
		self.text_field.setFont(self.newFont)
		self.text_field.resize(self.text_field.sizeHint())
		self.vlayout.addWidget(self.text_field)

		self.vlayout.addStretch()

		# Go back button
		self.back_btn = QtGui.QPushButton("Go Back")
		self.back_btn.setFont(self.newFont)
		self.back_btn.setObjectName("add_tts_behavior")
		self.vlayout.addWidget(self.back_btn)

		# Set up the layout
		self.setLayout(self.vlayout)


class execThread(threading.Thread):

	def __init__(self, name, lfd, mode):
		threading.Thread.__init__(self)
		self.name = name
		self.lfd = lfd
		self.mode = mode
		# Hard code the ticking rate
		self.sleep_rate = rospy.Rate(10)
		self.interrupt_flag = False

	def run(self):
		# Run the thread in different modes
		if self.mode == "Execute_dt":
			while not self.interrupt_flag:
				self.lfd.execute_dt()
				self.sleep_rate.sleep()
		elif self.mode == "Execute_CDNF":
			if self.lfd.bt_mode != None and self.lfd.bt_mode != 'CDNF':
				self.lfd.tree.blackboard_exchange.unregister_services()
			if self.lfd.bt_mode != 'CDNF':
				start = time()
				self.lfd.tree = self.lfd.get_bt(dt=self.lfd.clf.named_steps['classification'], bt_type='CDNF')
				stop = time()
				print 'CDNF time: ' + str(stop - start)
			while not self.interrupt_flag:
				self.lfd.execute()
				self.sleep_rate.sleep()
		elif self.mode == "Execute_naive_algo":
			if self.lfd.bt_mode != None and self.lfd.bt_mode != 'Naive':
				self.lfd.tree.blackboard_exchange.unregister_services()
			if self.lfd.bt_mode != 'Naive':
				start = time()
				self.lfd.tree = self.lfd.get_bt(dt=self.lfd.clf.named_steps['classification'], bt_type='Naive')
				stop = time()
				print 'Naive time: ' + str(stop - start)
			while not self.interrupt_flag:
				self.lfd.execute()
				self.sleep_rate.sleep()
		elif self.mode == "Execute_SOP":
			if self.lfd.bt_mode != None and self.lfd.bt_mode != 'SOP':
				self.lfd.tree.blackboard_exchange.unregister_services()
			if self.lfd.bt_mode != 'SOP':
				start = time()
				self.lfd.tree = self.lfd.get_bt(dt=self.lfd.clf.named_steps['classification'], bt_type='SOP')
				stop = time()
				print 'SOP time: ' + str(stop - start)
			while not self.interrupt_flag:
				self.lfd.execute()
				self.sleep_rate.sleep()
		elif self.mode == "Execute_BTEspresso":
			if self.lfd.bt_mode != None and self.lfd.bt_mode != 'Espresso':
				self.lfd.tree.blackboard_exchange.unregister_services()
			if self.lfd.bt_mode != 'Espresso':
				start = time()
				self.lfd.tree = self.lfd.get_bt(dt=self.lfd.clf.named_steps['classification'], bt_type='Espresso')
				stop = time()
				print 'Espresso time: ' + str(stop - start)
			while not self.interrupt_flag:
				self.lfd.execute()
				self.sleep_rate.sleep()

def main():
	# py_trees.logging.level = py_trees.logging.Level.DEBUG
	app = QtGui.QApplication(sys.argv)
	gui = LfDGui()
	nav_override_gui = NavOverrideGui()
	nav_override_gui.show()
	sys.exit(app.exec_())

if __name__ == '__main__':
	os.chdir(os.path.expanduser('~/catkin_ws/src/mobile_manipulation/lfd'))
	main()