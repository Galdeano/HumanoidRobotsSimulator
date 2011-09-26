
%% Basic Figure Setup

t=0:0.01:50;
X=8*sin(t/8);
Y=7*cos(t/3);

h1 = plot( t , X);
h2 = line( t , Y);

set( h1 , 'Color' , [0 0 1] );
set( h1 , 'Color' , [1 0 0] );

axis([0 50 -10 10]);
set( gca , 'XTick' , [0 : 5 : 50] );
set( gca , 'YTick' , [-10 : 2 : 10] );
xlabel( gca , 'Time [s]' );
ylabel( 'Signal Noise [V]' );

Figure2c



















