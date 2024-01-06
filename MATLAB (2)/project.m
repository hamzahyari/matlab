function varargout = project(varargin)

gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @project_OpeningFcn, ...
                   'gui_OutputFcn',  @project_OutputFcn, ...
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


% --- Executes just before project is made visible.
function project_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to project (see VARARGIN)

% Choose default command line output for project
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes project wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = project_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton9.
function pushbutton9_Callback(hObject, eventdata, handles)
A = imread('apple.jpg');

% Parameters for local Laplacian filter
    sigma = 0.4;
    alpha = 0.5;

    % Apply local Laplacian filter
    B = locallapfilt(A, sigma, alpha);

    % Get handles to the GUI components
    axes2Handle = findobj('Tag', 'axes2');  % Assuming you named your axes component 'axes2'

    % Display the filtered image on axes2
    imshow(B);
   



% --- Executes on button press in pushbutton10.
function pushbutton10_Callback(hObject, eventdata, handles)
im=imread("apple.jpg");
x=str2num(get(handles.edit1,'string'));
im_bw=im2bw(im,x);
axes(handles.axes2);
imshow(im_bw);


% --- Executes on button press in pushbutton11.
function pushbutton11_Callback(hObject, eventdata, handles)
% Load the image
    i = imread('apple.jpg');
    I1 = rgb2gray(i);
    i1 = histeq(I1);
    med1 = medfilt2(i1);
    imshow(med1);
    

% --- Executes on button press in pushbutton12.
function pushbutton12_Callback(hObject, eventdata, handles)

i=imread('apple.jpg');
w1=fspecial('average',[3 3]);
w2=fspecial('average',[5 5]);

j=imfilter(i,w1);
k=imfilter(i,w2,'replicate');

axes(handles.axes2);
imshow(j);


% --- Executes on button press in pushbutton13.
function pushbutton13_Callback(hObject, eventdata, handles)

% Load the image
i = imread('apple.jpg');

% Define the weighted average filter kernel
w3 = [-1, -1, -1; -1, 8, -1; -1, -1, -1];

g=imfilter(i,w3);

axes(handles.axes2);
imshow(g);


% --- Executes on button press in pushbutton16.
function pushbutton16_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton16 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
im=imread("apple.jpg");
axis(handles.axes1);
imagesc(im);



function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double


% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton17.
function pushbutton17_Callback(hObject, eventdata, handles)
 

 hsize = 7; 
sigma = 40; 

h = fspecial('gaussian', [hsize hsize], sigma);
img = imread('apple.jpg');
filtered_img = imfilter(img, h);
axes(handles.axes2);
imshow(filtered_img);


% --- Executes on button press in pushbutton18.
function pushbutton18_Callback(hObject, eventdata, handles)
i=imread('apple.jpg');
z=str2num(get(handles.edit2,'string'));
j=i*0;
[r c]=size(i);
for x=1:r
 for y=1:c
 g=bitget(i(x,y),z);
 j(x,y)=bitset(j(x,y),z,g);
 end
end
axis(handles.axes2);
imshow(j);


% --- Executes on button press in pushbutton19.
function pushbutton19_Callback(hObject, eventdata, handles)
i =imread('apple.jpg');
% Perform FFT and other operations
fi = fftshift(fft2(i));
[m, n] = size(i);
[x, y] = meshgrid(1:n, 1:m);

% Ensure the meshgrid is centered properly
x = x - floor(n/2);
y = y - floor(m/2);

D = sqrt(x.^2 + y.^2);

% Define the high-pass filter
D0 = 6;
IHPF = D > D0;

% Display the high-pass filter
axes(handles.axes1); % Use axes1 or another axes component for displaying the filter
imshow(IHPF);
title('High-Pass Filter');

% Apply the high-pass filter in the frequency domain
i_filt = fi .* IHPF;

% Perform inverse FFT
ii = ifft2(fftshift(i_filt));

% Display the filtered image
axes(handles.axes2);
imshow(abs(ii), []);
title('Filtered Image');

% --- Executes on button press in pushbutton20.
function pushbutton20_Callback(hObject, eventdata, handles)
img = imread('apple.jpg');

Ts = 1/50;
t = 0:Ts:10-Ts;
x = sin(2*3.1416*15*t) + sin(2*3.1416*20*t);

axis(handles.axes2);

imshow(x);


% Update handles structure
guidata(hObject, handles);


% --- Executes on button press in pushbutton21.
function pushbutton21_Callback(hObject, eventdata, handles)
a = imread('apple.jpg');
G = rgb2gray(a);
F = edge (G , 'sobel');
axis(handles.axes2);
imshow(F);
