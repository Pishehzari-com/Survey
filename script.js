(()=>{
const $=s=>document.querySelector(s),cv=$('#gl'),c=cv.getContext('2d'),R=matchMedia('(prefers-reduced-motion: reduce)').matches;
const S=2.6,B=5,AL=[.8,.55,.36,.2,.09];
let W,H,D,N,F,FAR,sky,vs,lastL;
const lt=()=>document.documentElement.dataset.theme==='light';
function mkSky(){const l=lt();sky=c.createLinearGradient(0,0,0,H);if(l){sky.addColorStop(0,'#c9ddf2');sky.addColorStop(.5,'#e1ecf8');sky.addColorStop(1,'#f1f5fa')}else{sky.addColorStop(0,'#0b1a30');sky.addColorStop(.5,'#06101e');sky.addColorStop(1,'#04070d')}}
const rs=()=>{D=Math.min(devicePixelRatio||1,2);W=innerWidth;H=innerHeight;cv.width=W*D;cv.height=H*D;c.setTransform(D,0,0,D,0,0);
 N=W<700?28:42;F=Math.max(H,W*.75)*.95;FAR=N*S*.95;vs=new Float32Array((N+17)*(N+1)*4);
 mkSky()};
addEventListener('resize',rs);rs();
const hf=(x,z)=>5*Math.sin(x*.11+Math.sin(z*.07)*1.6)+3.5*Math.cos(z*.13-x*.05)+1.6*Math.sin((x+z)*.31)+2*Math.abs(Math.sin(x*.045+z*.03));
const sm=t=>t*t*(3-2*t),pathX=z=>22*Math.sin(z/320*5.5);
const K=[[0,9,.38,0,0,0],[.2,7,.3,.35,0,0],[.4,15,.62,-.3,1,0],[.6,10,.45,.2,0,1],[.8,36,1.05,0,.5,1],[1,12,.3,0,0,0]];
const L=['AERIAL MESH','AERIAL MESH','POINT CLOUD','CONTOUR / DEM','ORTHO / CADASTRE','DELIVERY'];
const key=p=>{let i=0;while(i<K.length-2&&p>K[i+1][0])i++;const a=K[i],b=K[i+1],t=sm(Math.min(1,Math.max(0,(p-a[0])/(b[0]-a[0]))));return{v:a.map((x,k)=>x+(b[k]-x)*t),l:L[t>.5?i+1:i]}};
let cx,cy,cz,sY,cY,sA,cA;const o=[0,0,0];
const P=(x,y,z)=>{const dx=x-cx,dy=y-cy,dz=z-cz,x1=dx*cY-dz*sY,z1=dx*sY+dz*cY,z2=z1*cA-dy*sA;if(z2<.4)return false;
 const y2=dy*cA+z1*sA;o[0]=W/2+x1*F/z2;o[1]=H/2-y2*F/z2;o[2]=z2;return true};
const poly=(pts,close)=>{c.beginPath();let f=1;for(const p of pts){if(!P(p[0],p[1],p[2])){f=1;continue}f?c.moveTo(o[0],o[1]):c.lineTo(o[0],o[1]);f=0}if(close)c.closePath();c.stroke()};
let cur=0,tgt=0,T=0,fr=0,first=1,ty=0;
const sc=()=>{const m=document.documentElement.scrollHeight-innerHeight;tgt=m>0?Math.min(1,scrollY/m):0};
addEventListener('scroll',sc,{passive:true});sc();
const hud=$('#hud'),bar=$('#bar');
function frame(ts){
 requestAnimationFrame(frame);T=ts/1000;fr++;
 const Lt=lt();if(Lt!==lastL){lastL=Lt;mkSky()}
 const pal=Lt?{tl:a=>`rgba(14,116,110,${a*.9})`,dot:a=>`rgba(18,64,112,${a})`,hot:'rgba(217,119,6,.95)',amb:'#d97706',bm:a=>`rgba(190,98,0,${a})`,w:'#0b2545',tc:'#0f766e',fp1:'rgba(217,119,6,.4)',fp2:'rgba(217,119,6,.95)'}:{tl:a=>`rgba(62,224,208,${a})`,dot:a=>`rgba(190,235,255,${a})`,hot:'rgba(255,176,32,.95)',amb:'#ffb020',bm:a=>`rgba(255,176,32,${a})`,w:'#fff',tc:'#3ee0d0',fp1:'rgba(255,176,32,.35)',fp2:'rgba(255,176,32,.9)'};
 const prev=cur;cur+=(tgt-cur)*(R?1:.07);const vel=(cur-prev)*60,k=key(cur),v=k.v;
 cz=cur*320;cx=pathX(cz)+(R?0:Math.sin(T*.5)*.6);
 const yaw=Math.atan(22*5.5/320*Math.cos(cz/320*5.5))+v[3],A=v[2];sY=Math.sin(yaw);cY=Math.cos(yaw);sA=Math.sin(A);cA=Math.cos(A);
 const gr=Math.max(hf(cx,cz),hf(cx+sY*8,cz+cY*8));ty=gr+v[1]+2;cy=first?ty:cy+(ty-cy)*.1;first=0;
 // drone placement: on the view ray, slightly below centre
 const A2=A+.16,px=cx+sY*Math.cos(A2)*10,py=cy-Math.sin(A2)*10,pz=cz+cY*Math.cos(A2)*10,sr=R?20:(T*16)%70;
 c.fillStyle=sky;c.fillRect(0,0,W,H);
 // terrain
 const M=N,Wd=N+16,x0=Math.floor(cx/S)*S-Wd/2*S,z0=Math.floor(cz/S)*S-5*S;let n=0;
 for(let j=0;j<=M;j++)for(let i=0;i<=Wd;i++){const x=x0+i*S,z=z0+j*S;let h=hf(x,z);h+=(Math.round(h/2.4)*2.4-h)*v[5];
  if(P(x,h,z)&&o[2]<FAR){vs[n++]=o[0];vs[n++]=o[1];vs[n++]=o[2];vs[n++]=Math.abs(Math.hypot(x-px,z-pz)-sr)<1.8?1:0}else{vs[n++]=0;vs[n++]=0;vs[n++]=-1;vs[n++]=0}}
 const ln=[],dt=[];for(let b=0;b<B;b++){ln.push(new Path2D);dt.push(new Path2D)}const hot=new Path2D,hd=new Path2D;
 const seg=(a,b)=>{const A_=a*4,B_=b*4;if(vs[A_+2]<0||vs[B_+2]<0)return;const z=(vs[A_+2]+vs[B_+2])/2,bk=Math.min(B-1,z/FAR*B|0),p=vs[A_+3]?hot:ln[bk];p.moveTo(vs[A_],vs[A_+1]);p.lineTo(vs[B_],vs[B_+1])};
 for(let j=0;j<=M;j++)for(let i=0;i<=Wd;i++){const id=j*(Wd+1)+i;if(i<Wd)seg(id,id+1);if(j<M)seg(id,id+Wd+1);
  if(vs[id*4+2]>0){const r=1.7*(1-vs[id*4+2]/FAR)+.5,p=vs[id*4+3]?hd:dt[Math.min(B-1,vs[id*4+2]/FAR*B|0)];p.rect(vs[id*4]-r,vs[id*4+1]-r,2*r,2*r)}}
 c.lineWidth=1;
 if(v[4]<.95)for(let b=0;b<B;b++){c.strokeStyle=pal.tl(AL[b]*(1-v[4]));c.stroke(ln[b])}
 if(v[4]>.05)for(let b=0;b<B;b++){c.fillStyle=pal.dot(Math.min(1,AL[b]*1.4)*v[4]);c.fill(dt[b])}
 c.strokeStyle=pal.hot;c.lineWidth=1.8;c.stroke(hot);c.fillStyle=pal.amb;c.fill(hd);
 // survey control points
 c.font='11px monospace';
 for(let q=1;q<=7;q++){const mz=q*42,mx=pathX(mz)+7,g=hf(mx,mz);if(!P(mx,g,mz)||o[2]>FAR*1.1)continue;const bz=o[2],bx=o[0],by=o[1];
  const a=Math.max(0,1-bz/(FAR*1.1));c.strokeStyle=pal.bm(a);c.lineWidth=1.5;
  poly([[mx,g,mz],[mx,g+4.5,mz]]);poly([[mx,g+4.5,mz],[mx+2.2,g+3.9,mz],[mx,g+3.3,mz]],1);
  c.beginPath();c.ellipse(bx,by,1.4*F/bz,.5*F/bz,0,0,7);c.stroke();
  if(bz<70&&P(mx,g+5.2,mz)){c.fillStyle=pal.bm(a);c.fillText(`BM-0${q}  Z ${(1243+g*3.1).toFixed(1)}`,o[0]-30,o[1])}}
 // drone
 const ng=.12+Math.min(.3,Math.abs(vel)*3),cn=Math.cos(ng),sn=Math.sin(ng),rl=Math.sin(T*1.3)*.05,cr=Math.cos(rl),sr_=Math.sin(rl),by_=Math.sin(T*2)*.12;
 const Wd3=(lx,ly,lz)=>{const x=lx*cr-ly*sr_,y=lx*sr_+ly*cr,z=lz*cn+y*sn,yy=y*cn-lz*sn;return[px+x*cY+z*sY,py+yy+by_,pz-x*sY+z*cY]};
 const cube=(cxl,cyl,czl,w,h,d)=>{const q=[];for(const a of[-1,1])for(const b of[-1,1])for(const e of[-1,1])q.push(Wd3(cxl+a*w,cyl+b*h,czl+e*d));return q};
 const edges=q=>{for(const[a,b]of[[0,1],[2,3],[4,5],[6,7],[0,2],[1,3],[4,6],[5,7],[0,4],[1,5],[2,6],[3,7]])poly([q[a],q[b]])};
 c.shadowColor=pal.tc;c.shadowBlur=Lt?3:10;c.lineWidth=1.6;c.strokeStyle=pal.w;edges(cube(0,0,0,.55,.14,.55));
 for(const sx of[-1,1])for(const sz of[-1,1]){poly([Wd3(sx*.4,0,sz*.4),Wd3(sx*1.3,.05,sz*1.3)]);
  c.strokeStyle=pal.tc;const rc=[],rt=T*38+sx+sz*2;for(let t=0;t<=12;t++)rc.push(Wd3(sx*1.3+Math.cos(t/12*6.283)*.8,.16,sz*1.3+Math.sin(t/12*6.283)*.8));poly(rc);
  poly([Wd3(sx*1.3+Math.cos(rt)*.8,.16,sz*1.3+Math.sin(rt)*.8),Wd3(sx*1.3-Math.cos(rt)*.8,.16,sz*1.3-Math.sin(rt)*.8)]);c.strokeStyle=pal.w}
 c.strokeStyle=pal.amb;edges(cube(0,-.34,.45,.16,.16,.16));c.shadowBlur=0;
 // scan footprint
 const gp=Wd3(0,-.34,.6),fp=[];for(const[a,b]of[[-3.2,-2.6],[3.2,-2.6],[3.2,2.6],[-3.2,2.6]]){const gx=gp[0]+a*cY+b*sY,gz=gp[2]-a*sY+b*cY;fp.push([gx,hf(gx,gz)+.15,gz]);}
 c.strokeStyle=pal.fp1;c.lineWidth=1;for(const f of fp)poly([gp,f]);c.strokeStyle=pal.fp2;c.lineWidth=1.5;poly(fp,1);
 // HUD
 if(fr%4==0){hud.innerHTML=`<b>${k.l}</b><br>E ${(512340+cx*3.1).toFixed(1)}<br>N ${(3951200+cz*3.1).toFixed(1)}<br>Z ${(1243+gr*3.1).toFixed(1)} &nbsp; AGL ${(cy-gr).toFixed(1)} m<br>PROGRESS ${(cur*100).toFixed(0)}%`;bar.style.transform=`scaleY(${cur})`}
}
requestAnimationFrame(frame);
// reveal + counters
const num=new Intl.NumberFormat('fa-IR');
const io=new IntersectionObserver(es=>es.forEach(e=>{if(!e.isIntersecting)return;e.target.classList.add('in');e.target.querySelectorAll('[data-n]').forEach(el=>{const to=+el.dataset.n,t0=performance.now();const st=t=>{const p=Math.min(1,(t-t0)/1400);el.textContent=num.format(Math.round(to*(1-Math.pow(1-p,3))))+el.dataset.s;p<1&&requestAnimationFrame(st)};R?el.textContent=num.format(to)+el.dataset.s:requestAnimationFrame(st)});io.unobserve(e.target)}),{threshold:.15});
document.querySelectorAll('.rv').forEach(e=>io.observe(e));
// 3D tilt on cards (desktop pointers only)
if(matchMedia('(hover:hover)').matches&&!R)document.querySelectorAll('.card').forEach(el=>{
 el.addEventListener('pointermove',e=>{const r=el.getBoundingClientRect(),x=(e.clientX-r.left)/r.width-.5,y=(e.clientY-r.top)/r.height-.5;el.style.transition='transform .1s';el.style.transform=`perspective(900px) rotateY(${x*8}deg) rotateX(${-y*8}deg)`});
 el.addEventListener('pointerleave',()=>{el.style.transition='transform .6s';el.style.transform=''})});
})();

(()=>{
const nav=document.getElementById('nav'),bg=document.getElementById('burger'),pg=nav.querySelector('.pg'),links=[...nav.querySelectorAll('nav a')],secs=links.map(a=>document.querySelector(a.getAttribute('href')));
const set=o=>{nav.classList.toggle('open',o);bg.setAttribute('aria-expanded',o);document.body.style.overflow=o?'hidden':''};
bg.addEventListener('click',()=>set(!nav.classList.contains('open')));
links.forEach(a=>a.addEventListener('click',()=>set(false)));
addEventListener('keydown',e=>e.key==='Escape'&&set(false));
addEventListener('resize',()=>innerWidth>900&&set(false));
const up=()=>{const y=scrollY,m=document.documentElement.scrollHeight-innerHeight;nav.classList.toggle('stuck',y>30);pg.style.transform=`scaleX(${m>0?y/m:0})`;let a=0;secs.forEach((s,i)=>{if(s&&s.getBoundingClientRect().top<innerHeight*.4)a=i});links.forEach((l,i)=>l.classList.toggle('on',i===a))};
addEventListener('scroll',up,{passive:true});up();
const tb=document.getElementById('theme'),mt=document.querySelector('meta[name=theme-color]');
const paint=()=>{const l=document.documentElement.dataset.theme==='light';mt&&mt.setAttribute('content',l?'#eef3f9':'#04070d');tb.setAttribute('aria-pressed',l)};
paint();
tb.addEventListener('click',()=>{const n=document.documentElement.dataset.theme==='light'?'dark':'light';document.documentElement.dataset.theme=n;try{localStorage.setItem('theme',n)}catch(e){}paint()});
const f=document.getElementById('form'),st=document.getElementById('st'),WA='989921654985';
f.addEventListener('submit',e=>{e.preventDefault();
 const d=new FormData(f),n=(d.get('name')||'').trim(),c=(d.get('contact')||'').trim(),t=d.get('topic'),m=(d.get('msg')||'').trim();
 f.elements.name.classList.toggle('err',!n);f.elements.contact.classList.toggle('err',!c);
 if(!n||!c){st.textContent='لطفاً نام و راه ارتباطی را وارد کنید.';f.elements[n?'contact':'name'].focus();return}
 const text=`سلام مهندس پیشه‌زاری، وقت بخیر 👋\nدرخواست جدید از سایت:\n\n👤 نام و نام خانوادگی: ${n}\n📞 راه ارتباطی: ${c}\n📌 موضوع پروژه: ${t}\n📝 توضیح کوتاه: ${m||'—'}`;
 const a=document.createElement('a');a.href=`https://wa.me/${WA}?text=${encodeURIComponent(text)}`;a.target='_blank';a.rel='noopener';document.body.appendChild(a);a.click();a.remove();
 st.textContent='در حال باز کردن واتساپ… اگر باز نشد، دوباره دکمه را بزنید.'});
})();
