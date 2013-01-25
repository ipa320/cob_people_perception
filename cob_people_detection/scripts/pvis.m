
clear all;

sc1=[1,10,18];

col_vec={"ro","ko","bo"};

%clf(figure(1));
%clf(figure(2));

dir_path="/share/goa-tz/people_detection/eval/vis/";


fname_plda=[dir_path,"projFF"];
fname_ppca=[dir_path,"projEF"];
fname_sampleEF=[dir_path,"sampleEF"];
fname_sampleFF=[dir_path,"sampleFF"];

plda=load(fname_plda);
ppca=load(fname_ppca);
sample_pca=load(fname_sampleEF);
sample_lda=load(fname_sampleFF);

set(0, 'defaultfigurevisible', 'off');
for c=1:1:numel(sc1)-1

for(i=sc1(c):1:sc1(c+1)-1)
  figure(1)
  %%plot3(ppca(i,1),ppca(i,2),ppca(i,3),col_vec{c},'MarkerSize',10,'LineWidth',4)
  plot(ppca(i,1),ppca(i,2),col_vec{c},'MarkerSize',10,'LineWidth',4)
  hold on;
  figure(2)
  %%plot(plda(i,1),plda(i,2),    col_vec{c},'MarkerSize',10,'LineWidth',4)
  plot(plda(i),plda(i),col_vec{c},'MarkerSize',10,'LineWidth',4)
  hold on;
  end
end


figure(1)
plot(sample_pca(:,1),sample_pca(:,2),    'go','MarkerSize',10,'LineWidth',4)
hold on;
%%plot3(sample_pca(:,1),sample_pca(:,2),sample_pca(:,3),    'go','MarkerSize',10,'LineWidth',4)
hold on;

figure(2)
hold on;
%%plot(sample_lda(:,1),sample_lda(:,2),    'go','MarkerSize',10,'LineWidth',4)
plot(sample_lda(:),sample_lda(:),    'go','MarkerSize',10,'LineWidth',4)
hold on;


of1=[dir_path,"clustering_EF.jpg"];
of2=[dir_path,"clustering_FF.jpg"];
print(1,of1,"-djpg")
print(2,of2,"-djpg")
