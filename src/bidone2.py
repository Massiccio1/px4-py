import PySimpleGUI as sg

layout = [[
    sg.Frame('Input data',[[
          sg.Text('Input:'),      
          sg.Input(do_not_clear=False),      
          sg.Button('Read'), sg.Exit(),
          sg.Text('Alternatives:'),
          sg.Listbox(values=('alternatives...', ''), size=(30, 2), key='_LISTBOX_')
    ]]),
    sg.Frame('second data',[[
          sg.Text('Input:'),      
          sg.Input(do_not_clear=False),      
          sg.Button('Read'), sg.Exit(),
          sg.Text('Alternatives:'),
          sg.Listbox(values=('alternatives...', ''), size=(30, 2), key='_LISTBOX_')
    ]])
]]

window = sg.Window('App', layout)
event, values = window.Read()
window.Close()