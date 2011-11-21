\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{uva-bachelor-thesis}[2005/06/10 Version 0.1 Rein van den Boomgaard]
\RequirePackage{graphicx}

\usepackage[intoc]{nomencl}
\def\supervisors#1{\gdef\@supervisors{#1}}
\def\signedby#1{\gdef\@signedby{#1}}


\DeclareOption*{\PassOptionsToClass{\CurrentOption}{report}}
\ProcessOptions
\LoadClass[10t]{report}

%\usepackage[dutch]{babel}
\usepackage{graphicx}
\usepackage{url}
\usepackage{makeidx}
%\usepackage{hyperref}

\linespread{1.1}

\setcounter{tocdepth}{2}
\setcounter{secnumdepth}{2}

\usepackage[a4paper,scale={0.7,0.8},twoside]{geometry}


\oddsidemargin 0.0in 
\evensidemargin 0.0in 
\textwidth 6.0in 
\headheight 0.0in 
\topmargin 0.0in 
%\textheight 9.0in 
%\footheight 1.0in 

\usepackage[sf]{titlesec}
\titleformat{\chapter}[display]
{\ifodd\c@page \clearpage \else \fi \sffamily}
{\vskip-9em \Huge \thechapter}
{0mm}
{\Huge}
[\vspace{-1em}]

\usepackage{float}


\usepackage[absolute,overlay]{textpos}
\renewcommand{\maketitle}{%
	\thispagestyle{empty}
	\hspace*{1em} 
	
	\begin{textblock*}{21cm}(0cm,10cm)
	
	\begin{center}
	{\LARGE \sffamily \@title}
	
	\vspace{1.3cm}
	\sffamily \@author\\
	\@date
	
	\vspace{1.3cm}

	\includegraphics[scale=1.0]{images/UVALOGO.pdf}
	\end{center}
	
	\end{textblock*}	
	
	
	\begin{textblock*}{12cm}(3cm,25cm)
		\noindent\sffamily
		\begin{description}
		\item[\sffamily Bachelor Thesis:] Bachelor of Computer Science
		%\item[\sffamily Faculteit:] Faculteit der Natuurwetenschappen, Wiskunde en Informatica
		\item[\sffamily Supervisors:] \@supervisors
		\item[\sffamily Signed:] \@signedby
		\end{description}
	\end{textblock*}
	%\cleardoublepage

	
}

\usepackage{caption}
\renewcommand\captionfont{\itshape\small}

\usepackage{listings}
  \usepackage{courier}
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
\usepackage{caption}
\usepackage{color}
\DeclareCaptionFont{white}{\color{white}}
\DeclareCaptionFormat{listing}{\colorbox[cmyk]{0.43, 0.35, 0.35,0.01}{\parbox{\textwidth}{\hspace{15pt}#1#2#3}}}
\captionsetup[lstlisting]{format=listing,labelfont=white,textfont=white, singlelinecheck=false, margin=0pt, font={bf,footnotesize}}

\renewcommand{\nomname}{Lijst met afkortingen}

\endinput