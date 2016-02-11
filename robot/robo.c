#include <stdio.h>
int sx=0,sy=0;
int gx=5,gy=5;
int map_min=0,map_max=6;
void print();
int find_path(int a, int b, int d);
char map[6][6]={ 
    {'s','.','.','.','#','#'},
    {'#','.','.','.','#','#'},
    {'#','.','.','.','#','#'},
    {'#','.','.','.','.','.'},
    {'#','.','#','#','#','#'},
    {'#','.','.','.','.','g'}};
void print()
{    
    for(sx=0;sx<6;sx++)
    {
        for(sy=0;sy<6;sy++)
        {
            printf(" %c", map[sx][sy]);
        }
        printf("\n");
    }
    printf("\n");
    //sleep(1);
}
void main()
{
    print(); 
    if(find_path(0,0,1)==1)
        printf("Goal not reachable\n");    
    else
        printf("Goal reachable\n");    
    
    print(); 


}

int find_path(int a, int b, int d)
{
    if((a>map_max) || (b>map_max) || (a<0) || (b<0))
        return 1;

    if((a==gx) && (b==gy))
        return 0;

    if(map[a][b]=='#')
        return 1;
    
    if(map[a][b]=='+')
        return 1;

    map[a][b]='+';
    printf("Going %d\n", d);

    if(find_path(a-1,b,1) == 0)
    {
        return 0;
    }
    if(find_path(a,b+1,2) == 0)
    {
        return 0;
    }
    if(find_path(a+1,b,3) == 0)
    {
        return 0;
    }
    if(find_path(a,b-1,4) == 0)
    {
        return 0;
    }

    map[a][b]='x';
    return 1;

}

