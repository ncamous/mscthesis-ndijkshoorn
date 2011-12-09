\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{uva-bachelor-thesis}[2005/06/10 Version 0.1 Rein van den Boomgaard]

%\usepackage[a4paper,twoside]{geometry}
\LoadClass[10t]{book}


\def\supervisors#1{\gdef\@supervisors{#1}}
\def\signedby#1{\gdef\@signedby{#1}}
\def\email#1{\def\@email{\url{#1}}}


%\usepackage[intoc]{nomencl}
\usepackage[absolute]{textpos}
\usepackage{graphicx}
\usepackage{url}
\usepackage{makeidx}
\usepackage{verbatim} 
\usepackage{palatino}
\usepackage{amsmath}
\usepackage{ifthen}
\usepackage[rm]{titlesec}
\usepackage{caption}
\usepackage{listings}
\usepackage{courier}
\usepackage{caption}
\usepackage{color}
\usepackage{xcolor}


%\usepackage[T1]{fontenc}
%\usepackage{textcomp}
%\usepackage[default,lf,semibold]{MyriadPro}% Adobe Myriad Pro
%\usepackage[light,math]{iwona}


\linespread{1.5}

\setcounter{tocdepth}{2}
\setcounter{secnumdepth}{2}


\setlength{\oddsidemargin}{0.0in}
\setlength{\evensidemargin}{0.0in}
\setlength{\marginparwidth}{0.0in}
\setlength{\footskip}{0.6in}
\setlength{\parskip}{0pt}
\setlength{\parsep}{0pt}
\setlength{\headsep}{0.7in}
\setlength{\topskip}{0pt}
\setlength{\topmargin}{-1.2cm}
\setlength{\topsep}{0pt}
\setlength{\partopsep}{0pt}
\setlength{\headheight}{0in}
\textwidth 6.5in
\textheight 8.7in



% COLORS
\definecolor{darkgray}{rgb}{0.1,0.1,0.1}
\definecolor{lightgray}{rgb}{0.85,0.85,0.85}
\definecolor{mediumgray}{rgb}{0.2,0.2,0.2}
\definecolor{titlenumber}{rgb}{0.6,0.6,0.6}


\usepackage[
linkcolor=darkgray,
pdfborder=0 0 0,
colorlinks=true,
urlcolor=darkgray,
citecolor=darkgray,
]{hyperref}



% TITLE FORMAT
\titleformat{\chapter}[block]
{ \rmfamily
\ifthenelse{\equal{\thechapter}{0}}
{  \begin{textblock}{4}(11,1) \hfill \chapterNumber \color{titlenumber}  \end{textblock}  }
{  \begin{textblock}{4}(11,1) \hfill \chapterNumber \color{titlenumber} \thechapter \end{textblock}   }}
{}
{0cm}
{\Huge \emph}
[\vspace{0.5cm}]

\titleformat{\subsubsection}{\bf\normalsize}{\thesubsubsection}{1em}{}

\titlespacing*{\chapter}{0pt}{-0.35in}{0.1in}



% TITLE PAGE
\renewcommand{\maketitle}{%
	\thispagestyle{empty}	

	\begin{textblock*}{21cm}(0cm,10cm)
	
	\begin{center}
	{\LARGE \@title}
	
	\vspace{1.0cm}
	 \@author\\
	 %\@email\\
	\vspace{1.0cm}
	\@date
	
	\vspace{6.5cm}

	\includegraphics[scale=1.0]{images/UVALOGO.pdf}
	\end{center}
	
	\end{textblock*}	
	
	
	\begin{textblock*}{12cm}(3cm,25cm)
		\noindent
		\begin{description}
		\item[Master Thesis:] Artificial Intelligence
		%\item[Faculteit:] Faculteit der Natuurwetenschappen, Wiskunde en Informatica
		\item[Supervisors:] \@supervisors
		\item[Signed:] \@signedby
		\end{description}
	\end{textblock*}
	\hspace{0.1cm} % prevents page from being empty
	\cleardoublepage

	
}


\renewcommand\captionfont{\itshape\small}

 \lstset{
         basicstyle=\scriptsize\ttfamily, % Standardschrift
         numbers=left,               % Ort der Zeilennummern
         numberstyle=\tiny,          % Stil der Zeilennummern
         %stepnumber=2,               % Abstand zwischen den Zeilennummern
         numbersep=5pt,              % Abstand der Nummern zum Text
         tabsize=2,                  % Groesse von Tabs
         extendedchars=true,         %
         breaklines=true,            % Zeilen werden Umgebrochen
         keywordstyle=\color{red},
                frame=b,         
		keywordstyle=\color[rgb]{0,0,1},
        commentstyle=\color[rgb]{0.133,0.545,0.133},
        stringstyle=\color[rgb]{0.627,0.126,0.941},
         stringstyle=\it\ttfamily, % Farbe der String
         showspaces=false,           % Leerzeichen anzeigen ?
         showtabs=false,             % Tabs anzeigen ?
         xleftmargin=17pt,
         framexleftmargin=17pt,
         framexrightmargin=5pt,
         framexbottommargin=4pt,
         %backgroundcolor=\color{lightgray},
         showstringspaces=false      % Leerzeichen in Strings anzeigen ?        
 }
 \lstloadlanguages{Java}
    %\DeclareCaptionFont{blue}{\color{blue}} 

 %\captionsetup[lstlisting]{singlelinecheck=false, labelfont={blue}, textfont={blue}}
\DeclareCaptionFont{white}{\color{white}}
\DeclareCaptionFormat{listing}{\colorbox[cmyk]{0.43, 0.35, 0.35,0.01}{\parbox{\textwidth}{\hspace{15pt}#1#2#3}}}
\captionsetup[lstlisting]{format=listing,labelfont=white,textfont=white, singlelinecheck=false, margin=0pt, font={bf,footnotesize}}

% FONTS
\newfont{\chapterNumber}{eurb10 scaled 7000}


\newcommand{\cfbox}[2]{%
	\vspace{0.5cm}
    \colorlet{currentcolor}{.}%
    {\color{lightgray}%
    \fbox{\color{mediumgray}#2}}%
   	\vspace{0.5cm}
}


\endinput