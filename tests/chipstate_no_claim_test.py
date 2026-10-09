import os,subprocess,sys
new,spy,*baseline=sys.argv[1:]
def run(binary,args,**extra):
 env=os.environ.copy();env.update(LD_PRELOAD=spy,**extra)
 return subprocess.run([binary,'--pid','0xa81a','--no-claim',*args],env=env,text=True,capture_output=True)
peek=['--peek','0x0280-0x0288:4']
if baseline:
 r=run(baseline[0],peek);assert r.returncode==70,(r.returncode,r.stdout,r.stderr)
 print('BEFORE: real chipstate CLI attempts bulk-halt/interface mutation in no-claim mode')
r=run(new,peek);assert r.returncode==0 and '04002003 00020000 00000000' in r.stdout and 'mutations=0 closes=1 exits=1 reads=3 writes=0' in r.stderr,r
r=run(new,['--peek','0x0000']);assert r.returncode==0 and ' 44' in r.stdout,r
for width,value,bytes_ in [(1,'0xab','ab'),(2,'0x1234','3412'),(4,'0x89abcdef','efcdab89')]:
 r=run(new,['--poke',f'0x280={value}:{width}']);assert r.returncode==0 and f'SPY_WRITE bytes={bytes_}' in r.stdout and 'mutations=0' in r.stderr,r
for failure in ('BXI_SPY_SHORT','BXI_SPY_ERROR'):
 r=run(new,peek,**{failure:'1'});assert r.returncode==4 and '04002003' not in r.stdout,r
 r=run(new,['--poke','0x280=0xab:1'],**{failure:'1'});assert r.returncode==4 and 'poke 0x0280 =' not in r.stdout,r
r=run(new,['--init',*peek]);assert r.returncode==2 and 'reads=0 writes=0' in r.stderr and 'mutations=0' in r.stderr,r
print('AFTER: no implicit USB mutation; exact EP0 reads, LE writes, short/error rejection and invalid init cleanup PASS')
