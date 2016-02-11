
pariom : 
	g++ -o pariom robo_path_finder_pi.cpp -lpigpio -lpthread
clean : 
	rm -rf *.o *.out pariom 
