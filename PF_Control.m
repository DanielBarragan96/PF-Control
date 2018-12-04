function varargout = PF_Control(varargin)
% PF_CONTROL MATLAB code for PF_Control.fig
%      PF_CONTROL, by itself, creates a new PF_CONTROL or raises the existing
%      singleton*.
%
%      H = PF_CONTROL returns the handle to a new PF_CONTROL or the handle to
%      the existing singleton*.
%
%      PF_CONTROL('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in PF_CONTROL.M with the given input arguments.
%
%      PF_CONTROL('Property','Value',...) creates a new PF_CONTROL or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before PF_Control_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to PF_Control_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help PF_Control

% Last Modified by GUIDE v2.5 03-Dec-2018 19:03:40

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @PF_Control_OpeningFcn, ...
                   'gui_OutputFcn',  @PF_Control_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before PF_Control is made visible.
function PF_Control_OpeningFcn(hObject, eventdata, handles, varargin)
global bandera
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to PF_Control (see VARARGIN)
%Valores para la grafica
% Choose default command line output for PF_Control
handles.output = hObject;
% Update handles structure
guidata(hObject, handles);
bandera = 'cuad';

% UIWAIT makes PF_Control wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = PF_Control_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Get default command line output from handles structure
varargout{1} = handles.output;



function G_P_Callback(hObject, eventdata, handles)
% hObject    handle to G_P (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global G_P 
G_P = str2double(get(hObject,'String'));
if isnan(G_P)
    set(hObject, 'String', 0);
    errordlg('Lo que introdujo no es un valor numerico','Error');
end
% Hints: get(hObject,'String') returns contents of G_P as text
%        str2double(get(hObject,'String')) returns contents of G_P as a double


% --- Executes during object creation, after setting all properties.
function G_P_CreateFcn(hObject, eventdata, handles)
% hObject    handle to G_P (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function G_I_Callback(hObject, eventdata, handles)
% hObject    handle to G_I (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global  G_I 
G_I = str2double(get(hObject,'String'));
if isnan(G_I)
    set(hObject, 'String', 0);
    errordlg('Lo que introdujo no es un valor numerico','Error');
end
% Hints: get(hObject,'String') returns contents of G_I as text
%        str2double(get(hObject,'String')) returns contents of G_I as a double


% --- Executes during object creation, after setting all properties.
function G_I_CreateFcn(hObject, eventdata, handles)
% hObject    handle to G_I (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function G_D_Callback(hObject, eventdata, handles)
% hObject    handle to G_D (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global  G_D  
G_D = str2double(get(hObject,'String'));
if isnan(G_D)
    set(hObject, 'String', 0);
    errordlg('Lo que introdujo no es un valor numerico','Error');
end
% Hints: get(hObject,'String') returns contents of G_D as text
%        str2double(get(hObject,'String')) returns contents of G_D as a double


% --- Executes during object creation, after setting all properties.
function G_D_CreateFcn(hObject, eventdata, handles)
% hObject    handle to G_D (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function P_I_X_Callback(hObject, eventdata, handles)
% hObject    handle to P_I_X (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global  P_I_X 
P_I_X = str2double(get(hObject,'String'));
if isnan(P_I_X)
    set(hObject, 'String', 0);
    errordlg('Lo que introdujo no es un valor numerico','Error');
end
if (P_I_X<5)
    set(hObject, 'String', 0);
    errordlg('El minimo valor a medir es de 6','Error');
end
if (P_I_X>35)
    set(hObject, 'String', 0);
    errordlg('El maximo valor a medir es de 35','Error');
end
% Hints: get(hObject,'String') returns contents of P_I_X as text
%        str2double(get(hObject,'String')) returns contents of P_I_X as a double


% --- Executes during object creation, after setting all properties.
function P_I_X_CreateFcn(hObject, eventdata, handles)
% hObject    handle to P_I_X (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function P_I_Y_Callback(hObject, eventdata, handles)
% hObject    handle to P_I_Y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global  P_I_X P_I_Y
P_I_Y = str2double(get(hObject,'String'));
if isnan(P_I_X)
    set(hObject, 'String', 0);
    errordlg('Lo que introdujo no es un valor numerico','Error');
end
if (P_I_Y<5)
    set(hObject, 'String', 0);
    errordlg('El minimo valor a medir es de 6','Error');
end
if (P_I_Y>35)
    set(hObject, 'String', 0);
    errordlg('El maximo valor a medir es de 35','Error');
end
% Hints: get(hObject,'String') returns contents of P_I_Y as text
%        str2double(get(hObject,'String')) returns contents of P_I_Y as a double


% --- Executes during object creation, after setting all properties.
function P_I_Y_CreateFcn(hObject, eventdata, handles)
% hObject    handle to P_I_Y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in tria.
function tria_Callback(hObject, eventdata, handles)
% hObject    handle to tria (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global Bandera_triangulo 
Bandera_triangulo = get(hObject,'Value');
set(handles.distancia_1,'String','  ');
set(handles.distancia_2,'String','  ');


% Hint: get(hObject,'Value') returns toggle state of tria


% --- Executes on button press in circ.
function circ_Callback(hObject, eventdata, handles)
% hObject    handle to circ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global Bandera_Circulo 
Bandera_Circulo = get(hObject,'Value');
set(handles.distancia_1,'String','  ');
set(handles.distancia_2,'String','  ');


% Hint: get(hObject,'Value') returns toggle state of circ



function distancia_1_Callback(hObject, eventdata, handles)
% hObject    handle to distancia_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global distancia_1    
distancia_1 = str2double(get(hObject,'String'));

% Hints: get(hObject,'String') returns contents of distancia_1 as text
%        str2double(get(hObject,'String')) returns contents of distancia_1 as a double


% --- Executes during object creation, after setting all properties.
function distancia_1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to distancia_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function distancia_2_Callback(hObject, eventdata, handles)
% hObject    handle to distancia_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global distancia_2  
distancia_2 = str2double(get(hObject,'String'));


% --- Executes during object creation, after setting all properties.
function distancia_2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to distancia_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in Limpiar.
function Limpiar_Callback(hObject, eventdata, handles)
% hObject    handle to Limpiar (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
set(handles.G_P,'String','  ');
set(handles.G_I,'String','  ');
set(handles.G_D,'String','  ');
set(handles.distancia_1,'String','  ');
set(handles.distancia_2,'String','  ');
set(handles.P_I_X,'String','  ');
set(handles.P_I_Y,'String','  ');
plot(handles.axes3, zeros(1,2), zeros(1,2));
xlim([0 35]);
ylim([0 35]);


% --- Executes on slider movement.
function slider2_Callback(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider4_Callback(hObject, eventdata, handles)
% hObject    handle to slider4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over circ.
function circ_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to circ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on key press with focus on circ and none of its controls.
function circ_KeyPressFcn(hObject, eventdata, handles)
% hObject    handle to circ (see GCBO)
% eventdata  structure with the following fields (see MATLAB.UI.CONTROL.UICONTROL)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in cuad.
function cuad_Callback(hObject, eventdata, handles)
% hObject    handle to cuad (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global Bandera_triangulo 
Bandera_triangulo = get(hObject,'Value');
set(handles.distancia_1,'String','  ');
set(handles.distancia_2,'String','  ');
% Hint: get(hObject,'Value') returns toggle state of cuad


% --- Executes when selected object is changed in uibuttongroup2.
function uibuttongroup2_SelectionChangedFcn(hObject, eventdata, handles)
% hObject    handle to the selected object in uibuttongroup2 
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global bandera 
bandera = get(eventdata.NewValue,'Tag');
switch(get(eventdata.NewValue,'Tag'))
    case 'cuad'
        set(handles.text10,'Visible','on');
        set(handles.text10,'String','BASE');
        set(handles.text11,'Visible','on');
        set(handles.text11,'String','Altura');
        set(handles.text18,'String','Para el cuadrado');
        set(handles.distancia_1,'Visible','on');
        set(handles.distancia_2,'Visible','on');
    case 'tria'
        set(handles.text10,'String','LADO');
        set(handles.text11,'Visible','off');
        set(handles.text10,'Visible','off');
        set(handles.distancia_1,'Visible','off');
        set(handles.distancia_2,'Visible','off');
        set(handles.text18,'String','Para el triángulo la distancia es 4');
    case 'circ'
        set(handles.text10,'Visible','on');
        set(handles.text10,'String','RADIO X');
        set(handles.text11,'Visible','on');
        set(handles.text18,'String','Para el círculo');
        set(handles.text11,'String','RADIO Y');
        set(handles.distancia_1,'Visible','on');
        set(handles.distancia_2,'Visible','on');
end


% --- Executes on button press in Enviar.
function Enviar_Callback(hObject, eventdata, handles)
% hObject    handle to Enviar (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global distancia_1 distancia_2 P_I_X P_I_Y
global P1X_T P2X_T P3X_T P4X_T P1Y_T P2Y_T P3Y_T P4Y_T P1X_C P2X_C P3X_C P4X_C P5X_C 
global P1Y_C P2Y_C P3Y_C P4Y_C P5Y_C  G_P G_I G_D bandera 
    plot(handles.axes3, zeros(1,2), zeros(1,2));
    xlim([0 35]);
    ylim([0 35]);
    G_P = G_P;   
    G_I = G_I;
    G_D = G_D;
    delete(instrfind({'Port'},{'COM10'}));
    puerto_serial=serial('COM10');
    puerto_serial.Baudrate=9600;
    warning('off','MATLAB:serial:fscanf:unsuccessfulRad');
    fopen(puerto_serial);
    pause(3);
    if(bandera == 'tria')
        lado_t = 4;
        P1X_T = P_I_X;
        P2X_T = P_I_X + lado_t;
        P3X_T = P_I_X + lado_t/2;
        P4X_T = P_I_X;
        P1Y_T = P_I_Y;
        P2Y_T = P_I_Y;
        P3Y_T = P_I_Y + lado_t/2;
        P4Y_T = P_I_Y;
        if ((P2X_T > 35))
            set(hObject, 'String', 0);
            errordlg('Pasa los valores de la maqueta','Error');
        end
        if ((P3Y_T > 35))
            set(hObject, 'String', 0);
            errordlg('Pasa los valores de la maqueta','Error');
        end
        fwrite(puerto_serial,G_P);
        fwrite(puerto_serial,G_I);
        fwrite(puerto_serial,G_D);
        fwrite(puerto_serial,4);
        fwrite(puerto_serial,P1X_T);
        fwrite(puerto_serial,P2X_T);
        fwrite(puerto_serial,P3X_T);
        fwrite(puerto_serial,P4X_T);
        fwrite(puerto_serial,P1Y_T);
        fwrite(puerto_serial,P2Y_T);
        fwrite(puerto_serial,P3Y_T);
        fwrite(puerto_serial,P4Y_T);
        
    elseif(bandera == 'cuad')
        P1X_C = P_I_X;
        P2X_C = P_I_X + distancia_1;
        P3X_C = P2X_C;
        P4X_C = P3X_C - distancia_1;
        P5X_C = P_I_X;
        P1Y_C = P_I_Y;
        P2Y_C = P_I_Y;
        P3Y_C = P_I_Y + distancia_2;
        P4Y_C = P3Y_C;
        P5Y_C = P_I_Y;

        if ((P2X_C > 35))
            set(hObject, 'String', 0);
            errordlg('Pasa los valores de la maqueta','Error');
        end
        if ((P3Y_C > 35))
            set(hObject, 'String', 0);
            errordlg('Pasa los valores de la maqueta','Error');
        end
        fwrite(puerto_serial,G_P);
        fwrite(puerto_serial,G_I);
        fwrite(puerto_serial,G_D);
        fwrite(puerto_serial,5);
        
        fwrite(puerto_serial,P1X_C);
        fwrite(puerto_serial,P2X_C);
        fwrite(puerto_serial,P3X_C);
        fwrite(puerto_serial,P4X_C);
        fwrite(puerto_serial,P5X_C);

        fwrite(puerto_serial,P1Y_C);
        fwrite(puerto_serial,P2Y_C);
        fwrite(puerto_serial,P3Y_C);
        fwrite(puerto_serial,P4Y_C);
        fwrite(puerto_serial,P5Y_C);
    end
eje_x = 0;
eje_y = 0;
exit = 1;
i = 1;
while (exit)
    eje_x(i) = fread(puerto_serial,1);
    eje_y(i) = fread(puerto_serial,1);
    if 50 == eje_x(i)
        exit=0;
        break;
    else
        plot(handles.axes3, eje_x, eje_y);
        xlim([0 35]);
        ylim([0 35]);
        grid; 
        set(handles.slider2,'Value', (eje_x(i)/35));
        set(handles.slider4,'Value', (eje_y(i)/35));
        set(handles.text20,'String', 'eje_x(i)');
        set(handles.text19,'String', 'eje_x(i)');
    end
    i=i+1;
    pause(0.1);
end
pause(1);
fclose(puerto_serial);
