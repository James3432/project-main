#define isnan(x) (x!=x)

float nan()
{
	unsigned long nan[2]={0xffffffff, 0x7fffffff}; 
	return *( float* )nan;
}