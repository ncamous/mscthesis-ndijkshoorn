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
\usepackage{subfigure}
\usepackage{amsthm}
\usepackage{dsfont}


\newtheoremstyle{slplain}% name
  {.3\baselineskip\@plus.2\baselineskip\@minus.2\baselineskip}% Space above
  {.3\baselineskip\@plus.2\baselineskip\@minus.2\baselineskip}% Space below
  {\itshape\rmfamily}% Body font
  {}%Indent amount (empty = no indent, \parindent = para indent)
  {\bfseries\rmfamily}%  Thm head font
  {. }%       Punctuation after thm head
  { }%      Space after thm head: " " = normal interword space;
        %       \newline = linebreak
  {}%       Thm head spec


\theoremstyle{slplain}

\newtheorem{name}{Printed output}
\newtheorem{mydef}{Definition}

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
{  \begin{textblock}{4}(10.70,1) \hfill \chapterNumber \color{titlenumber}  \end{textblock}  }
{  \begin{textblock}{4}(10.70,1) \hfill \chapterNumber \color{titlenumber} \thechapter \end{textblock}   }}
{}
{0cm}
{\Huge \emph}
[\vspace{0.5cm}]

\titleformat{\subsubsection}{\bf\normalsize}{\thesubsubsection}{1em}{}

\titlespacing*{\chapter}{0pt}{-0.35in}{0.1in}



% TITLE PAGE
\renewcommand{\maketitle}{%
	\thispagestyle{empty}	

	\begin{textblock*}{21cm}(0cm,4cm)
	
	\begin{center}
	{\LARGE \@title}
	
	\vspace{0.5cm}
	 \@author\\
	 %\@email\\
	\vspace{0.4cm}
	\@date
	
	\vspace{3.0cm}

	\includegraphics[height=5cm]{images/ardrone_blender_final.png}

	\end{center}
	
	\end{textblock*}	
	
	
	\begin{textblock*}{11cm}(3cm,22.5cm)
		\noindent
		\includegraphics[scale=1.0]{images/UVALOGO.pdf}

		\begin{description}
		\item[Master's Thesis] for the graduation in Artificial Intelligence
		%\item[Faculteit:] Faculteit der Natuurwetenschappen, Wiskunde en Informatica
		\item[Supervised by] \@supervisors
		%\item[Signed:] \@signedby
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


\renewcommand{\paragraph}{\@startsection{paragraph}{4}{0ex}%
   {-3.25ex plus -1ex minus -0.2ex}%
   {1.5ex plus 0.2ex}%
   {\normalfont\normalsize\bfseries}}


\DeclareSymbolFont{AMSb}{U}{msb}{m}{n}
\DeclareMathSymbol{\N}{\mathbin}{AMSb}{"4E}
\DeclareMathSymbol{\Z}{\mathbin}{AMSb}{"5A}
\DeclareMathSymbol{\R}{\mathbin}{AMSb}{"52}
\DeclareMathSymbol{\Q}{\mathbin}{AMSb}{"51}
\DeclareMathSymbol{\I}{\mathbin}{AMSb}{"49}
\DeclareMathSymbol{\C}{\mathbin}{AMSb}{"43}


\endinput