#include<conio.h>
#include<stdio.h>
#include<alloc.h>

typedef struct vertex
{
	int *label;
	int plabel;
	int *adjvertex;
	int *adjedge;
	int flag;
}ver;

int min(int *edge,int p)
{
	int min,j;
	min=edge[0];
	for(j=0;j<p;j++)
	if(edge[j]<=min)
	min=edge[j];
	return min;
}

void main()
{
	ver *v=NULL;
	int i,j,n,k,p,q,ch,s,d,m,t,*edge,pos,*perlev,l,perval,min1;
	
	clrscr();
	printf("\nenter the no of vertex\n");
	
	scanf("%d",&n);
	v=(ver*)calloc(n,sizeof(ver));
	
	for(i=0;i<n;i++)
	{
		v[i].label=(int *)calloc(n,sizeof(int));
		v[i].adjvertex=(int *)calloc(n,sizeof(int));
		v[i].adjedge=(int *)calloc(n,sizeof(int));
		v[i].label[0]=1000;
		v[i].plabel=-1;
	}
	
	for(i=0;i<n;i++)
	{
		for(j=0;j<n;j++)
		{
			v[i].label[j]=1000;
		}
		v[i].flag=0;
	}
	
	edge=(int*)calloc(n,sizeof(int));
	perlev=(int*)calloc(n,sizeof(int));
	printf("\nenter the adjacent vertex & the corresponding adjacent edges\n");
	
	for(i=0;i<n;i++)
	{
		printf("enter the values for ver%d\n",i);
		for(j=0;j<n;j++)
		{
			printf("ch->1 if connected or ch->0 if not connected ver %d to ver %d\n",i,j);
			scanf("%d",&ch);
			v[i].adjvertex[j]=ch;
			if(ch==1)
			{
				printf("enter all the parallel edges between vertex %d and vertex %d",i,j);
				l=0;
				do
				{
					scanf("%d",&edge[l]);
					l++;
				}while(edge[l-1]!=-1);
				v[i].adjedge[j]=min(edge,l-1);
			}
			else
			v[i].adjedge[j]=1000;
		}
	}
	
	for(i=0;i<n;i++)
	{
		printf("\n v[%d]",i);
		for(j=0;j<n;j++)
		printf("\t %d",v[i].adjedge[j]);
	}
	
	printf("\nenter the starting vertex & the destination vertex\n");
	scanf("%d %d",&s,&d);
	v[s].plabel=0;m=0;
	perval=0;v[s].flag=1;perlev[0]=s;
	v[s].label[0]=0;
	v[s].label[1]=0;
	t=s;
	
	while(v[d].plabel==-1)
	{
		for(i=0;i<n;i++)
		{
			printf("\n the value of flag of ver%d is %d\n",i,v[i].flag);
			if(v[i].flag==0)
			{
				k=perval+(v[t].adjedge[i]);
				if(v[i].label[m]>k)
					v[i].label[m+1]=k;
				else
					v[i].label[m+1]=v[i].label[m];
			}
			if(v[i].flag==1)
				v[i].label[m+1]=v[i].label[m];
		}
		for(i=0;i<=m;i++)
		{
			for(j=0;j<n;j++)
			{
				printf(" %d\t",v[j].label[i+1]);
			}
			printf("\n");
		}

		for(i=0;i<n;i++)
		{
			if(v[i].flag==0)
			{
				min1=v[i].label[m+1];
				break;
			}
		}
		
		for(;i<n;i++)
		{
			if(v[i].flag==0&&min1>=v[i].label[m+1])
			{
				min1= v[i].label[m+1];
				pos=i;
			}
		}
		m++;
		printf("the value of min1 is %d\n",min1);
		v[pos].plabel=min1;
		v[pos].flag=1;
		t=pos;
		perlev[m]=pos;
		perval=min1;
	}
	
	printf("\n ");
	
	for(i=0;i<n;i++)
		printf(" v%d\t",i);
		
	printf("\n");
	
	for(i=0;i<=m;i++)
	{
		for(j=0;j<n;j++)
		{
			printf(" %d\t",v[j].label[i]);
		}
		printf("\n");
	}
	
	printf("the length of shortest path is %d\n",v[perlev[m]].label[m]);
	printf("\n the shortest path is ");
	
	for(j=m;j>=0;)
	{
		q=j;
		printf(" vertex%d<-",perlev[j]);
		while(v[perlev[q]].label[j]==v[perlev[q]].label[j-1])
		j--;
		j--;
	}
	getch();
}
