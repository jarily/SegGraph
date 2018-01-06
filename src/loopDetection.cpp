#include "SegGraph.h"

void creatMap(std::vector<std::vector<float > > v1);

float ans[2000][2000];
int vis1[200],vis2[200];

float map1[200][200],map2[200][200];
float pi = 3.1415926;
int sum=0;
int edge1[10000][2];
int edge2[10000][2];
int edgecnt;
float eps=1.0;
int TAU = -1;

struct Seq
{
    int a1[200];
    int a2[200];
    int cnt;
} T[3000];

float dist(float x1,float y1,float z1,float x2,float y2,float z2)
{
    return sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));
}


std::vector<std::vector<float > > getData(char *dataFile)
{
    std::vector<std::vector<float > > v;
    std::ifstream f(dataFile);
    if (! f.is_open())
    {
        std::cout << "Error opening file";
        return v;
    }
    int i;
    int j=0;
    while (!f.eof())
    {
        float x1,y1,z1,x2,y2,z2,r,sum;
        f>>i>>x1>>y1>>z1>>x2>>y2>>z2>>r>>sum;
        if(j>i)
        {
            break;
        }
        std::vector<float> v1;
        v1.push_back(x1);
        v1.push_back(y1);
        v1.push_back(z1);
        v1.push_back(x2);
        v1.push_back(y2);
        v1.push_back(z2);
        v1.push_back(r);
        v1.push_back(sum);
        v.push_back(v1);
        j++;
    }
    // printf("i==%d , j==%d\n",i,j);
    f.close();
    return v;
}


int judgeNormal(float x,float y,float z)
{
    // printf(" (%f %f %f) ",x,y,z);
    float tmp;
    int flag = -1;
    fabs(x) > fabs(y) ? flag = 0 : flag = 1;
    if(flag == 0)
    {
        tmp = fabs(x);
    }
    else if(flag == 1)
    {
        tmp = fabs(y);
    }
    if(tmp < fabs(z))
    {
        flag = 2;
    }
    return flag;
}



void creatMap(std::vector<std::vector<float > > v1,std::vector<std::vector<float > > v2)
{
    for (int i = 0; i < v1.size(); i++)
    {
        for (int j = i; j < v1.size(); j++)
        {
            if(i==j)
            {
                map1[i][j]=0;
            }
            else
            {
                map1[i][j]=map1[j][i]=dist(v1[i][0],v1[i][1],v1[i][2],v1[j][0],v1[j][1],v1[j][2]);
            }
        }

    }


    for (int i = 0; i < v2.size(); i++)
    {
        for (int j = i; j < v2.size(); j++)
        {
            if(i==j)
            {
                map2[i][j]=0;
            }
            else
            {
                map2[i][j]=map2[j][i]=dist(v2[i][0],v2[i][1],v2[i][2],v2[j][0],v2[j][1],v2[j][2]);
            }
        }
    }

}

bool findVertex(int a[],int cnt,int tar)
{
    for(int i=0; i<cnt; i++)
    {
        if(a[i]==tar)
            return true;
    }
    return false;
}

void searchRecursion(int a1[],int a2[],int cnt,std::vector<std::vector<float > > v1,std::vector<std::vector<float > > v2)
{
    if(cnt>=TAU)
    {
        sum=cnt;
        return ;
    }
    int flag=0;
    for(int i=0; i<edgecnt; i++)
    {
        int tmp1_1=edge1[i][0];
        int tmp1_2=edge1[i][1];
        int tmp2_1=edge2[i][0];
        int tmp2_2=edge2[i][1];
        bool judge1 = findVertex(a1,cnt,tmp1_1);
        bool judge2 = findVertex(a1,cnt,tmp1_2);

        if(judge1&&judge2==false)
        {
            if(findVertex(a2,cnt,tmp2_1)==true&&findVertex(a2,cnt,tmp2_2)==false)
            {
                int counts=0;
                for(int j=0; j<cnt; j++)
                {
                    float diff= fabs(map1[tmp1_2][a1[j]]-map2[tmp2_2][a2[j]]);
                    if(diff<eps)
                        counts++;
                }
                if(counts==cnt)
                {
                    for(int k=0; k<cnt; k++)
                    {
                        T[flag].a1[k]=a1[k];
                        T[flag].a2[k]=a2[k];

                    }
 
                    T[flag].a1[cnt]=tmp1_2;
                    T[flag].a2[cnt]=tmp2_2;
                    T[flag].cnt=cnt+1;
                    flag++;

                }
            }
        }
        else if(judge2&&judge1==false)
        {
            if(findVertex(a2,cnt,tmp2_1)==false&&findVertex(a2,cnt,tmp2_2)==true)
            {
                int counts=0;
                for(int j=0; j<cnt; j++)
                {
                    float diff= fabs(map1[tmp1_1][a1[j]]-map2[tmp2_1][a2[j]]);
                    if(diff<eps)
                        counts++;
                }
                if(counts==cnt)
                {
                    for(int k=0; k<cnt; k++)
                    {
                        T[flag].a1[k]=a1[k];
                        T[flag].a2[k]=a2[k];

 
                    }

                    T[flag].a1[cnt]=tmp1_1;
                    T[flag].a2[cnt]=tmp2_1;
                    T[flag].cnt=cnt+1;
                    flag++;

                }
            }
        }
    }
    if(flag==0)
        return;

    srand((unsigned)time(NULL));
    int tar = rand()%flag;
    searchRecursion(T[tar].a1,T[tar].a2,T[tar].cnt,v1,v2);
}

void searchVertex(std::vector<std::vector<float > > v1,std::vector<std::vector<float > > v2)
{
    int a1[100],a2[100];
    int cnt=0;

    for(int i=0; i<edgecnt; i++)
    {
        cnt=0;
        a1[cnt]=edge1[i][0];
        a2[cnt]=edge2[i][0];
        cnt++;
        a1[cnt]=edge1[i][1];
        a2[cnt]=edge2[i][1];
        cnt++;
        searchRecursion(a1,a2,cnt,v1,v2);

    }
}

void searchEdge(int e1,int e2,std::vector<std::vector<float > > v1,std::vector<std::vector<float > > v2)
{
    int cnt = 0;
    float e = map1[e1][e2];
    float eps=1.0;
    int flag1 = judgeNormal(v1[e1][3],v1[e1][4],v1[e1][5]);
    int flag2 = judgeNormal(v1[e2][3],v1[e2][4],v1[e2][5]);
    for(int i=0; i<v2.size(); i++)
    {
        for (int j=i+1; j<v2.size(); j++)
        {
            float tmp = fabs(e-map2[i][j]);
            cnt=0;
            if(tmp<eps)
            {
                int flag3 = judgeNormal(v2[i][3],v2[i][4],v2[i][5]);
                int flag4 = judgeNormal(v2[j][3],v2[j][4],v2[j][5]);
                float d1 = dist(v1[e1][0],v1[e1][1],v1[e1][2],v2[i][0],v2[i][1],v2[i][2]);//两个点云簇的图心距离
                float d2 = dist(v1[e1][0],v1[e1][1],v1[e1][2],v2[j][0],v2[j][1],v2[j][2]);
                float d3 = dist(v1[e2][0],v1[e2][1],v1[e2][2],v2[i][0],v2[i][1],v2[i][2]);
                float d4 = dist(v1[e2][0],v1[e2][1],v1[e2][2],v2[j][0],v2[j][1],v2[j][2]);
                if(d1<5&&d4<5&&flag1==flag3&&flag2==flag4)
                {
                    edge1[edgecnt][0]=e1;
                    edge2[edgecnt][0]=i;
                    edge1[edgecnt][1]=e2;
                    edge2[edgecnt][1]=j;
                    edgecnt++;
                }
                else if(d2<5&&d3<5&&flag1==flag4&&flag2==flag3)
                {

                    edge1[edgecnt][0]=e2;
                    edge2[edgecnt][0]=i;
                    edge1[edgecnt][1]=e1;
                    edge2[edgecnt][1]=j;
                    edgecnt++;
                }
            }
        }

    }
}

int searchMatch(std::vector<std::vector<float > > v1,std::vector<std::vector<float > > v2)
{
    for(int i=0; i<v1.size(); i++)
    {
        for (int j=i+1; j<v1.size(); j++)
        {
            searchEdge(i,j,v1,v2);
        }

    }
    return 0;
}



bool loopDetection(char *feaFile1,char *feaFile2,int tau)
{
	TAU = tau;

    std::vector<std::vector<float > > v1 =getData(feaFile1);
    std::vector<std::vector<float > > v2 =getData(feaFile2);

    sum=0;
	edgecnt = 0;
    creatMap(v1,v2);//建图
    searchMatch(v1,v2);//求两个图中所有能够匹配的边
    searchVertex(v1,v2);//递归求解公共子图
	if (sum == tau)
	{
		printf("Loop Detect Success!\n");
		return true;
	}

	else
	{
		printf("Loop Detect Failed!\n");
		return false;
	}

}
