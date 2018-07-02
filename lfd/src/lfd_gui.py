#!/usr/bin/env python

import sys
from lfd import *
from PyQt4 import QtGui, QtCore

class LfDGui(QtGui.QMainWindow):

	def __init__(self):
		super(LfDGui, self).__init__()
		# lfd instance
		self.lfd = LfD()

		# Set up the basic frame
		self.setGeometry(50, 50, 400, 450)
		self.setFixedSize(400, 450)

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

		# Show the home interface
		self.display_home()

	def display_home(self):
		# Home layout widget
		home_layout = Home()

		# Set up button connections
		home_layout.btn_d.clicked.connect(self.display_action)
		home_layout.btn_l.clicked.connect(self.learn_cb)
		home_layout.btn_e.clicked.connect(self.execute_cb)
		home_layout.btn_r.clicked.connect(self.display_action)
		home_layout.btn_lm.clicked.connect(self.loadmodel_cb)
		home_layout.btn_wm.clicked.connect(self.writemodel_cb)
		home_layout.btn_u.clicked.connect(self.undo_cb)

		# Show the layout
		self.setGeometry(200, 200, 500, 300)
		self.setWindowTitle("Learning from Demonstration")
		self.setWindowIcon(QtGui.QIcon('lfd_logo.png'))
		self.setCentralWidget(home_layout)
		self.show()

	def display_action(self):
		# TODO: 
		# All button setting should be done
		# in the constructor of ActionInterface, not here

		# Action interface layout
		action_interface_layout = ActionInterface()

		# Choose callback function
		if self.sender().objectName() == "Demonstrate":
			button_cb = self.demonstrate_cb
		elif self.sender().objectName() == "Rc":
			button_cb = self.rc_cb
		else:
			button_cb = None

		# Action buttons
		for key, name in self.lfd.action_names.iteritems():
			button = QtGui.QPushButton(name)
			button.setObjectName(name)
			button.clicked.connect(button_cb)
			button.setFont(action_interface_layout.newFont)
			action_interface_layout.vlayout.addWidget(button)

		action_interface_layout.vlayout.addStretch()

		# Label
		action_interface_layout.back_label = QtGui.QLabel()
		action_interface_layout.back_label.setText("Go Back:")
		action_interface_layout.back_label.setFont(action_interface_layout.menu_font)

		action_interface_layout.vlayout.addWidget(action_interface_layout.back_label)

		# Go Back Button
		action_interface_layout.back_btn = QtGui.QPushButton("Go Back")
		action_interface_layout.back_btn.setFont(action_interface_layout.newFont)
		action_interface_layout.vlayout.addWidget(action_interface_layout.back_btn)

		# Set up go back button connections
		action_interface_layout.back_btn.clicked.connect(self.display_home)

		# Show the layout
		self.setCentralWidget(action_interface_layout)
		self.show()

	def display_exec(self):
		# Execute interface layout
		exec_interface = ExecuteInterface()
		
		# Add connections
		exec_interface.interrupt_btn.clicked.connect(self.lfd.tree.interrupt)
		exec_interface.back_btn.clicked.connect(self.display_home)
		
		# Show the layout
		self.setCentralWidget(exec_interface)
		self.show()

	def undo_cb(self):
		if self.lfd.demo_states is not None and len(self.lfd.demo_states) != 0:
			self.lfd.demo_states = self.lfd.demo_states[:-1]
			self.lfd.demo_actions = self.lfd.demo_actions[:-1]
		QtGui.QMessageBox.information(self, 'Success', 'Success in undoing the action!')


	def writemodel_cb(self):
		name = QtGui.QFileDialog.getSaveFileName(self, 'Save File')
		pickle.dump(self.lfd.clf, open(name, 'wb'))
		QtGui.QMessageBox.information(self, 'Success', 'Success in saving the model!')

	def loadmodel_cb(self):
		name = QtGui.QFileDialog.getOpenFileName(self, 'Open File')
		self.lfd.clf = pickle.load(open(name, 'rb'))
		self.lfd.render_model()
		QtGui.QMessageBox.information(self, 'Success', 'Success in loading the model!')


	def rc_cb(self):
		action_name = self.sender().objectName()
		self.lfd.run_action(action_name)
		QtGui.QMessageBox.information(self, 'Success', 'Success in performing ' + action_name +'!')

	def execute_cb(self):
		if self.lfd.demo_actions != None:
			execute_flag = True
		else:
			execute_flag = False
		if execute_flag == False:
			QtGui.QMessageBox.warning(self,"Error",  
                             self.tr("The robot haven't learned anything yet!"))
		else:
			# Show the interface
			self.display_exec()
			self.lfd.execute()

	def learn_cb(self):
		self.lfd.learn(self.lfd.demo_states, self.lfd.demo_actions)
		QtGui.QMessageBox.information(self, 'Success', 'The robot has finished the learning!')

	def demonstrate_cb(self):
		world_state = self.lfd.get_state()
		print 'World state is'
		self.lfd.print_state(world_state)
		action_name = self.sender().objectName()
		user_input = numpy.array([[int(self.lfd.action_indices[action_name])]])
		self.lfd.run_action(action_name)
		# Save the state action combo
		if self.lfd.demo_states is not None:
			self.lfd.demo_states = numpy.append(self.lfd.demo_states, world_state, axis=0)
			self.lfd.demo_actions = numpy.append(self.lfd.demo_actions, user_input)
		else:
			self.lfd.demo_states = world_state
			self.lfd.demo_actions = user_input

		QtGui.QMessageBox.information(self, 'Success', 'Success in performing ' + action_name +'!')

	def closeEvent(self, event):
		choice = QtGui.QMessageBox.question(self, 'Exit Prompt', 
											"Do you want to close LfD Gui?", 
											QtGui.QMessageBox.Yes | QtGui.QMessageBox.No)
		if choice == QtGui.QMessageBox.Yes:
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
		self.menu_label.setText("Meanu:")
		self.menu_label.setFont(self.menu_font)

		self.vlayout.addWidget(self.menu_label)
		self.vlayout.addStretch()

		# Buttons for users
		self.btn_d = QtGui.QPushButton("Demonstrate", self)
		self.btn_d.resize(self.btn_d.minimumSizeHint())
		self.btn_d.setObjectName("Demonstrate")
		self.btn_d.setFont(self.newFont)
		self.btn_d.move(0, 100)

		self.btn_l = QtGui.QPushButton("Learn", self)
		self.btn_l.resize(self.btn_l.minimumSizeHint())
		self.btn_l.setObjectName("Learn")
		self.btn_l.setFont(self.newFont)
		self.btn_l.move(0, 100)

		self.btn_e = QtGui.QPushButton("Execute", self)
		self.btn_e.resize(self.btn_e.minimumSizeHint())
		self.btn_e.setObjectName("Execute")
		self.btn_e.setFont(self.newFont)
		self.btn_e.move(0, 100)

		self.btn_r = QtGui.QPushButton("Rc", self)
		self.btn_r.resize(self.btn_r.minimumSizeHint())
		self.btn_r.setObjectName("Rc")
		self.btn_r.setFont(self.newFont)
		self.btn_r.move(0, 100)

		self.btn_lm = QtGui.QPushButton("Load model", self)
		self.btn_lm.resize(self.btn_lm.minimumSizeHint())
		self.btn_lm.setObjectName("Load model")
		self.btn_lm.setFont(self.newFont)
		self.btn_lm.move(0, 100)

		self.btn_wm = QtGui.QPushButton("Write model", self)
		self.btn_wm.resize(self.btn_wm.minimumSizeHint())
		self.btn_wm.setObjectName("Write model")
		self.btn_wm.setFont(self.newFont)
		self.btn_wm.move(0, 100)

		self.btn_u = QtGui.QPushButton("Undo last demo", self)
		self.btn_u.resize(self.btn_u.minimumSizeHint())
		self.btn_u.setObjectName("Undo last demo")
		self.btn_u.setFont(self.newFont)
		self.btn_u.move(0, 100)

		self.vlayout.addWidget(self.btn_d)
		self.vlayout.addWidget(self.btn_l)
		self.vlayout.addWidget(self.btn_e)
		self.vlayout.addWidget(self.btn_r)
		self.vlayout.addWidget(self.btn_lm)
		self.vlayout.addWidget(self.btn_wm)
		self.vlayout.addWidget(self.btn_u)

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
		self.vlayout.addStretch()

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
		self.interrupt_btn = QtGui.QPushButton('Stop execution')
		self.interrupt_btn.setFont(self.newFont)
		self.interrupt_btn.resize(self.interrupt_btn.sizeHint())
		self.vlayout.addWidget(self.interrupt_btn)

		self.vlayout.addStretch()

		# Label
		self.back_label = QtGui.QLabel()
		self.back_label.setText("Go Back:")
		self.back_label.setFont(self.menu_font)
		self.vlayout.addWidget(self.back_label)

		self.back_btn = QtGui.QPushButton('Go back')
		self.back_btn.setFont(self.newFont)
		self.back_btn.resize(self.back_btn.sizeHint())
		self.vlayout.addWidget(self.back_btn)

		self.setLayout(self.vlayout)

def main():
	app = QtGui.QApplication(sys.argv)
	gui = LfDGui()
	sys.exit(app.exec_())

if __name__ == '__main__':
	main()