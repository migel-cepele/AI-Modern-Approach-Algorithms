#Genetic algorithm search
import bisect
import numpy as np
from State import State


class GA:
    def __init__(self, population):
        self.population = population

    def genetic_search(self, ngen=10, pmut=0.1, n=10):

        states = self.population
        np.random.shuffle(states)

        return self.genetic_algorithm(states)



    def genetic_algorithm(self, population : list[State], gene_pool=[0,1], f_thres = None, ngen = 10, pmut = 0.1):
        for i in range(ngen):
            population = [self.mutate(self.recombine(*self.select(2, population)), gene_pool, pmut)
                          for i in range(len(population))]
            
            fittest_individual = self.fitness_threshhold(f_thres, population)
            if fittest_individual:
                return fittest_individual
            
        return np.argmax([state.value for state in population])

    
    def fitness_threshhold(self, f_thres, population : list[State]):
        if not f_thres:
            return None
        fittest_individual : State = np.argmax([state.value for state in population])
        if fittest_individual.value >= f_thres:
            return fittest_individual
        
        return None


    def init_population(self, pop_no, gene_pool, state_length):
        # pop_no is number of individuals in population
        # gene pool is list of possible values for individuals
        # state length is the length of each individual

        g = len(gene_pool)
        population = []
        for i in range(pop_no):
            new_individual = [gene_pool[np.random.randint(0,g)] for j in range(state_length)]
            population.append(new_individual)

        return population

    def select(self, r, population : list[State]):
        fitnesses = [item.value for item in population]
        sampler = self.weighted_sampler(population, fitnesses)
        return [sampler() for i in range(r)]
    

    def weighted_sampler(self,seq, weights):
        
        totals = []
        for w in weights:
            totals.append(w + totals[-1] if totals else w)
        return lambda: seq[bisect.bisect(totals, np.random.uniform(0, totals[-1]))]


    def recombine(self, xx: State, yy: State):
        x = xx.name
        y = yy.name
        n = len(x)
        c = np.random.randint(0, n)
        
        return x[:c] + y[c:]


    def recombine_uniform(self, x, y):
        n = len(x)
        result = [0] * n # list with size n with all 0
        indexes = np.random.sample(range(n), n) # returns ndarray with size n

        for i in range(n):
            ix = indexes[i]
            result[ix] = x[ix] if i < n/2 else y[ix]

        return ''.join(str(r) for r in result)

    def mutate(self, x, gene_pool, pmut):
        if np.random.uniform(0,1) >= pmut:
            return x
        
        n = len(x)
        g = len(gene_pool)
        c = np.random.randint(0,n)
        r = np.random.randint(0,g)

        new_gene = gene_pool[r]
        return x[:c] + new_gene + x[c+1:]
        
        

init_population = [
    State("ATCGBTACG", 20), 
    State("GTCGATCGT", 50), 
    State("TGCATGCAT", 30), 
    State("CGTAGCTAG", 100), 
    State("TAGCTAGCA", 85), 
    State("AGCTAGCTA", 90),
    State("CTAGCTAGC", 55),
    State("GATCGATCG", 60),
    State("TCGATCGAT", 20),
    State("CGATCGATC", 50)
    ]

ga = GA(init_population)
print(ga.genetic_search())